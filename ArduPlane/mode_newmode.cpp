#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

bool ModeNewmode::does_auto_throttle() const {
    return true; 
}

bool ModeNewmode::_enter()
{
    plane.prev_WP_loc = plane.current_loc; 
    current_wp_idx = 0;
    setup_ok = false;

    target_center_loc.lat = 346299857;
    target_center_loc.lng = 1092087713;
    
    Location wp_runway_start;
    wp_runway_start.lat = 346304216;
    wp_runway_start.lng = 1092083032;

    Location wp_runway_end;
    wp_runway_end.lat = 346294593;
    wp_runway_end.lng = 1092084762;

    runway_heading_cd = wp_runway_start.get_bearing_to(wp_runway_end);
    float dist_to_target = wp_runway_start.get_distance(target_center_loc);
    int32_t bearing_to_target = wp_runway_start.get_bearing_to(target_center_loc);
    float angle_rad = radians((bearing_to_target - runway_heading_cd) * 0.01f);
    runway_y_pos_m = - (dist_to_target * sinf(angle_rad));

    float right_y = runway_y_pos_m + 200.0f; 
    
    test_route_locs[0] = get_loc_from_target(  350.0f, runway_y_pos_m, 50.0f); 
    test_route_locs[1] = get_loc_from_target(  350.0f, right_y,        50.0f); 
    test_route_locs[2] = get_loc_from_target( -350.0f, right_y,        50.0f); 
    test_route_locs[3] = get_loc_from_target( -350.0f, runway_y_pos_m, 50.0f); 
    
    // 生成固定的虚拟起飞引航点，死死钉在前方 800 米
    takeoff_target_loc = plane.current_loc;
    takeoff_target_loc.offset_bearing(runway_heading_cd * 0.01f, 800.0f);
    takeoff_target_loc.set_alt_cm(5000, Location::AltFrame::ABOVE_HOME); 

    route_wp_count = 4;
    setup_ok = true;

    gcs().send_text(MAV_SEVERITY_INFO, "Wide Pattern Ready! Target WP0.");

    state = TestState::TAKEOFF_RUN; 
    return true;
}

Location ModeNewmode::get_loc_from_target(float x_m, float y_m, float alt_m)
{
    Location loc = target_center_loc; 
    float distance_m = sqrtf(x_m * x_m + y_m * y_m);
    float angle_deg = degrees(atan2f(y_m, x_m)); 
    
    int32_t final_bearing_cd = wrap_360_cd(runway_heading_cd + angle_deg * 100.0f);
    loc.offset_bearing(final_bearing_cd * 0.01f, distance_m);
    loc.set_alt_cm(alt_m * 100.0f, Location::AltFrame::ABOVE_HOME); 
    return loc;
}

void ModeNewmode::navigate_to_waypoint(const Location &loc)
{
    plane.next_WP_loc = loc;
    plane.set_target_altitude_location(plane.next_WP_loc); 

    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc); 
    
    // 巡航阶段正常调用三件套，交给 TECS 和 L1 大脑
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeNewmode::update()
{
    if (!setup_ok) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        return;
    }

    switch (state) {
        case TestState::TAKEOFF_RUN:
        {
            plane.set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);
            plane.throttle_suppressed = false;

            // 1. 导航目标锁定为钉死的虚拟航点
            plane.next_WP_loc = takeoff_target_loc;
            plane.set_target_altitude_location(plane.next_WP_loc);
            plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);

            // 2. 暴力硬件接管：油门满，横滚0，仰角15度
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
            plane.nav_roll_cd = 0;
            plane.nav_pitch_cd = 1500; 

            // 3. 【听你的建议：斩断 TECS！】 
            // 只调用 L1 横滚控制器来保持中心线，彻底封杀 calc_nav_pitch 和 calc_throttle！
            plane.calc_nav_roll();

            float dist_from_start = plane.current_loc.get_distance(plane.prev_WP_loc);

            if (plane.relative_altitude > 25.0f || dist_from_start > 150.0f) {
                // 平滑交接起点
                plane.prev_WP_loc = plane.current_loc; 
                state = TestState::FOLLOW_PATH; 
                gcs().send_text(MAV_SEVERITY_INFO, "Takeoff clear! Cruising.");
            }
            break;
        }

        case TestState::FOLLOW_PATH:
        {
            plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
            
            // 【听你的建议：真正的目标空速锁定】
            // 锁定巡航速度为 15m/s，防止速度过快冲过航点导致画大圆
           

            Location target_loc = test_route_locs[current_wp_idx];
            navigate_to_waypoint(target_loc);

            float dist = plane.current_loc.get_distance(target_loc);
            bool passed_line = plane.current_loc.past_interval_finish_line(plane.prev_WP_loc, target_loc);
            
            if (dist < plane.get_wp_radius() || passed_line) {
                plane.prev_WP_loc = target_loc;
                current_wp_idx++;
                gcs().send_text(MAV_SEVERITY_INFO, "WP %d Reached!", current_wp_idx);
                
                if (current_wp_idx >= route_wp_count) {
                    current_wp_idx = 0; 
                    gcs().send_text(MAV_SEVERITY_INFO, "Lap complete! Continuing...");
                }
            }
            break;
        }
    }
}