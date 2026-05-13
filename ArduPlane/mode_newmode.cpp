//这版的目的是，写一系列航点，调整参数去参看飞行效果。还是通过手动起飞。飞一遍，不频繁自主起降。
#include "mode.h"
#include "Plane.h"

bool ModeNewmode::_enter()
{
    reset_controllers();

    index = 0;                                  //归零索引
    num_waypoint[0].alt = 3000;
    num_waypoint[0].lng = 1092084816;
    num_waypoint[0].lat = 346292418;
    num_waypoint[0].relative_alt = 1;

    num_waypoint[1].alt = 3000;
    num_waypoint[1].lng = 1092094848;
    num_waypoint[1].lat = 346293501;
    num_waypoint[1].relative_alt = 1;

    num_waypoint[2].alt = 3000;
    num_waypoint[2].lng = 1092091334;
    num_waypoint[2].lat = 346307405;
    num_waypoint[2].relative_alt = 1;

    num_waypoint[3].alt = 3000;
    num_waypoint[3].lng = 1092081437;
    num_waypoint[3].lat = 346306390;
    num_waypoint[3].relative_alt = 1;

    plane.prev_WP_loc = plane.current_loc;   //把切入这个模式时的位置设为起点
     
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

    float turn_dist = plane.nav_controller->turn_distance(arrived_bool_radius);

    float switch_dist = MAX(arrived_bool_radius, turn_dist);

    bool passed_line = plane.current_loc.past_interval_finish_line(plane.prev_WP_loc, target_pos);


    if ((dist <= switch_dist) || passed_line)
    {
        if (index < NUM_WAYPOINTS - 1) {
            index++;
        } else {
            index = 0;
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
    creat_nav_waypoint();
    arrive_waypoint();
    fly_black_box();

}
//加一个航点到达判定。切换
//获取目标点


