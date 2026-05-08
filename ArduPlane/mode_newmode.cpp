#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

bool ModeNewmode::_enter()
{
    reset_controllers(); 
    plane.prev_WP_loc = plane.current_loc; 
    current_wp_idx = 0;
    setup_ok = false;

    // 1. 硬编码输入已知的三点绝对经纬度 (脱离底层读取，绝对稳定)
    // 经纬度统一放大 10^7 倍转为 ArduPilot 底层的 int32_t 格式
 
    
    // WP1: 靶心 (原点)
    target_center_loc.lat = 346299857;
    target_center_loc.lng = 1092087713;
    
    // WP2: 跑道起点 (靠近靶子的一侧)
    Location wp_runway_start;
    wp_runway_start.lat = 346304216;
    wp_runway_start.lng = 1092083032;

    // WP3: 跑道终点 
    Location wp_runway_end;
    wp_runway_end.lat = 346294593;
    wp_runway_end.lng = 1092084762;


    // 2. 几何解算：构建“靶心为原点的空间右手系”
    
    
    // A. 跑道向量 -> 绝对 X 轴方向
    runway_heading_cd = wp_runway_start.get_bearing_to(wp_runway_end);

    // B. 跑道在靶心坐标系中的 Y 轴坐标推演
    float dist_to_target = wp_runway_start.get_distance(target_center_loc);
    int32_t bearing_to_target = wp_runway_start.get_bearing_to(target_center_loc);
    float angle_rad = radians((bearing_to_target - runway_heading_cd) * 0.01f);
    
    // 跑道的 Y 坐标 = -(靶心相对于跑道的横向偏差)
    runway_y_pos_m = - (dist_to_target * sinf(angle_rad));//算出的是靶心到跑道的距离

    
    // 3. 动态生成 4点矩形测试航线
    
    
    // 顺风边建在跑道内侧10米处也就是中间
    float downwind_y = runway_y_pos_m - -5.0f;
    
    // 航点0：跑道前方 100m，顺风边 Y
    dynamic_wp_list[0] = {  100.0f, downwind_y, 20.0f };//(100,跑道中心,40)米
    // 航点1：掉头往回飞，靶心后方 200m，顺风边 Y
    dynamic_wp_list[1] = { -100.0f, downwind_y, 20.0f };
    // 航点2：向靶心靠拢，靶心后方 200m，Y 对准 0 (即靶心所在直线)
    dynamic_wp_list[2] = { -100.0f,       0.0f, 20.0f };
    // 航点3：平飞穿过靶心上空，靶心前方 200m，Y 依然是 0
    dynamic_wp_list[3] = {  100.0f,       0.0f, 20.0f };
    
    route_wp_count = 4;
    setup_ok = true;

    gcs().send_text(MAV_SEVERITY_INFO, "Test Field Ready! Runway_Y: %.1fm", (-runway_y_pos_m));

    state = TestState::TAKEOFF_RUN; 
    return true;
}

// 坐标转换黑盒：(X, Y, 相对高度) -> 地球绝对 Location
Location ModeNewmode::get_loc_from_target(float x_m, float y_m, float alt_m)
{
    Location loc = target_center_loc; 
    float distance_m = sqrtf(x_m * x_m + y_m * y_m);
    float angle_deg = degrees(atan2f(y_m, x_m)); 
    
    int32_t final_bearing_cd = wrap_360_cd(runway_heading_cd + angle_deg * 100.0f);
    loc.offset_bearing(final_bearing_cd * 0.01f, distance_m);
    loc.alt = plane.ahrs.get_home().alt + (alt_m * 100.0f); // 加上 Home 海拔
    
    return loc;//返回目标点的一个经纬度
}

void ModeNewmode::navigate_to_waypoint(const Location &loc)
{
    plane.next_WP_loc = loc;
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc); //L1导航   
    plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeNewmode::update()
{

    // 飞手摇杆抢舵拦截器 (最高优先级)
    /*
        好思路但是先不考虑这个。
        你不动拨杆（依然在 NEWMODE），直接用右手猛打一下副翼摇杆。
        代码瞬间触发 if (abs(rc_roll) > 450)。代码暂停了自动航线，把飞机的副翼交给你。你灵巧地躲开了鸟。
        躲开之后，你松开右手，摇杆弹簧自动回中 (abs(rc_roll) < 450)。
        奇迹发生了！ 代码立刻跳过抢舵拦截，继续往下执行 FOLLOW_PATH。因为你没切模式，游标 current_wp_idx 还在第三边。飞机就像什么都没发生过一样，极其平滑地继续去飞第四边投弹了！
     */

    if (!setup_ok) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        return;
    }

    // 极简状态机测试流

    switch (state) {
        case TestState::TAKEOFF_RUN:
        {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
            plane.nav_roll_cd = 0;
            plane.nav_pitch_cd = 1000; 

            // 2. 测算距离起飞点(Home)的直线距离
            float dist_from_home = plane.current_loc.get_distance(plane.ahrs.get_home());

            // 爬升达到 30米，或者飞了100米。切入航线网
            if (plane.relative_altitude > 10.0f|| dist_from_home > 50.0f) {//
                plane.prev_WP_loc = plane.current_loc; 
                state = TestState::FOLLOW_PATH; 
                gcs().send_text(MAV_SEVERITY_INFO, "Takeoff done, tracking rectangle.");
            }
            break;
        }

        case TestState::FOLLOW_PATH:
        {

            const TargetWaypoint &wp = dynamic_wp_list[current_wp_idx];
            Location target_loc = get_loc_from_target(wp.x_m, wp.y_m, wp.alt_m);
            navigate_to_waypoint(target_loc);

            float dist = plane.current_loc.get_distance(target_loc);
            
            // 抵达航点圈
            if (dist < plane.get_wp_radius()) {
                current_wp_idx++;
                gcs().send_text(MAV_SEVERITY_INFO, "yidaohangdian");
                
                //飞完第4个点，自动切回第1个点
                if (current_wp_idx >= route_wp_count) {
                    current_wp_idx = 0; 
                    gcs().send_text(MAV_SEVERITY_INFO, "Lap complete! Continuing...");
                }
                
                plane.prev_WP_loc = target_loc;
            }
            break;
        }
    }
}
