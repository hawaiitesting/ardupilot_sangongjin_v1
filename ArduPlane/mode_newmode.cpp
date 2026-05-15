//这版的目的是，写一系列航点，调整参数去参看飞行效果。还是通过手动起飞。飞一遍，不频繁自主起降。
#include "mode.h"
#include "Plane.h"

void ModeNewmode::add_waypoint(int32_t alt, int32_t lng, int32_t lat, bool rel_alt)
{
    // 安全保护：如果没超过最大容量，就把数据塞进去
    if (total_waypoints < MAX_WAYPOINTS) {
        num_waypoint[total_waypoints].alt          = alt;
        num_waypoint[total_waypoints].lng          = lng;
        num_waypoint[total_waypoints].lat          = lat;
        num_waypoint[total_waypoints].relative_alt = rel_alt;
        
        // 装完一个，总数自动加 1！
        total_waypoints++; 
    }
}

bool ModeNewmode::_enter()
{
    reset_controllers();

    index = 0;           // 当前飞向第 0 个点
    total_waypoints = 0; // 每次切入模式，清空重置有效航点数
    //航点
//=====================================================================
add_waypoint(3000, 1092084897, 346294671, 1);
add_waypoint(3000, 1092097396, 346296679, 1);
add_waypoint(3000, 1092096913, 346312084, 1);
add_waypoint(3000, 1092086023, 346311378, 1);
add_waypoint(3000, 1092088491, 346295046, 1);
add_waypoint(3000, 1092098254, 346296105, 1);
add_waypoint(3000, 1092096430, 346310936, 1);
add_waypoint(3000, 1092081302, 346311598, 1);
add_waypoint(3000, 1092082965, 346299151, 1);



//=====================================================================    

    plane.prev_WP_loc = plane.current_loc;   //把切入这个模式时的位置设为起点
    home_los          = plane.current_loc;   //拿到home点的坐标

    gcs().send_text(MAV_SEVERITY_INFO, "mode newmode");

    state = TestState::TAKEOFF_RUN;
    return true;
}

void ModeNewmode::creat_nav_waypoint()          // cm和*10的7次方
{
    target_pos.alt = num_waypoint[index].alt;
    target_pos.lng = num_waypoint[index].lng;
    target_pos.lat = num_waypoint[index].lat;
    target_pos.relative_alt = num_waypoint[index].relative_alt;
}

bool ModeNewmode::arrive_waypoint()
{
    float dist = plane.current_loc.get_distance(target_pos);
    //这个会得到当前点到目标点的距离
    float turn_dist = plane.nav_controller->turn_distance(arrived_bool_radius);
    //这个会得到根据气动和我们设置的半径，给我们一个提前转弯半径。这个转弯半径是提前转弯半径
    float switch_dist = MAX(arrived_bool_radius, turn_dist);//1.8f 是给个量，让他打的更狠

    bool passed_line = plane.current_loc.past_interval_finish_line(plane.prev_WP_loc, target_pos);
    //这个是终点线判定。

    if ((dist <= switch_dist) || passed_line)
    {
        if (index < total_waypoints - 1) {
            index++;
            gcs().send_text(MAV_SEVERITY_INFO, "WP %d finish, next %d", index, index + 1);
        } else {
            index = 0;
            gcs().send_text(MAV_SEVERITY_INFO, "All WPs finish, restart from WP 1");
        }
        
        plane.prev_WP_loc = target_pos; 
        
        return true;
    }
    
    return false;
}

void ModeNewmode::fly_black_box()
{
    plane.next_WP_loc = target_pos;
    plane.nav_controller->update_waypoint(plane.prev_WP_loc,plane.next_WP_loc);//L1导航控制器算出需要多大的向心力才能到达目标点。
    
    //plane.target_airspeed_cm = 1200;//12米每秒  //在这里设置之后，再地面站上设置的值就没用了

    plane.calc_nav_pitch();
    plane.calc_nav_roll();//把L1算出的向心力转化为飞机需要倾斜多少度，存到plane.nav_roll_cd这里,，，，，期望角
    plane.calc_throttle();//飞机转弯时会掉速，函数获取目标横滚，提前输出油门，使之不掉速。

    //将计算好的导航值；输出到舵机
    plane.stabilize_pitch();//输入（期望角），算出来执行机构（舵机需要输出多少度）
    plane.stabilize_roll();
    plane.set_servos();//输入（舵机需要输出多少度），，输出到舵机
}

void ModeNewmode::run()
{

    update();
}

void ModeNewmode::update()
{

    switch (state) {
        case TestState::TAKEOFF_RUN :
        {
            plane.nav_roll_cd = 0;
            plane.nav_pitch_cd = 1500;
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);

            plane.stabilize_pitch();
            plane.stabilize_roll();
            plane.set_servos();

            if(plane.current_loc.alt - home_los.alt >= 500 ||plane.current_loc.get_distance(home_los) >= 15)//高度5米和距离20米，同时判定
            {
              state = TestState::FOLLOW_PATH;
                
            }
            break;
        }
        case TestState::FOLLOW_PATH :
        {
            creat_nav_waypoint();
            arrive_waypoint();
            fly_black_box();
            break;            
        }
    }
}
//加一个航点到达判定。切换
//获取目标点


