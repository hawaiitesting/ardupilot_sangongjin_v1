/*
这版的目的是，写一个简单的飞向航点的，通过手动起飞，遥控切换，查看航点飞行情况，同时学习代码。
*/

#include "mode.h"
#include "Plane.h"
bool ModeNewmode::_enter()
{
    reset_controllers();

    Location current_pos = plane.current_loc;//把切入这个模式时的位置设为起点

    plane.prev_WP_loc = current_pos;

    return true;
}
void ModeNewmode::run()
{
    update();
    //将计算好的导航值；输出到舵机
    plane.stabilize_pitch();//输入（期望角），算出来执行机构（舵机需要输出多少度）
    plane.stabilize_roll();

    plane.set_servos();//输入（舵机需要输出多少度），，输出到舵机

}
void ModeNewmode::update()
{
    Location target_pos;
    target_pos.alt = 3000;
    target_pos.lng = 1092091441;
    target_pos.lat = 346294649;
    target_pos.relative_alt = true;//定义目标点，西航操场靠南一点30米高处。

    plane.next_WP_loc = target_pos;

    plane.nav_controller->update_waypoint(plane.prev_WP_loc,plane.next_WP_loc);//L1导航控制器算出需要多大的向心力才能到达目标点。
    
    plane.calc_nav_pitch();
    plane.calc_nav_roll();//把L1算出的向心力转化为飞机需要倾斜多少度，存到plane.nav_roll_cd这里,，，，，期望角
    plane.calc_throttle();//飞机转弯时会掉速，函数获取目标横滚，提前输出油门，使之不掉速。

}
