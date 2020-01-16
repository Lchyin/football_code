#include "GetBall.h"
#include "utils/maths.h"
#include "utils/constants.h"
#include "utils/PlayerTask.h"
#include "utils/worldmodel.h"
//#include "def.h"
#include <fstream>
#include <iostream>
#define debug 1
extern "C"_declspec(dllexport) PlayerTask player_plan(const WorldModel* model, int robot_id, int receiver_id);

double spiral_buff = 8.0;		// spiral 螺旋/盘绕
double get_ball_buf = -4;
double do_spiral_dist = 30;
double do_spiral_buff = 0;
double around_ball_dist = 30;
double vision_error = 3;
bool isSimulation = false;
int do_spiral_max_cnt = 70;
#define frame_rate  60.0
float away_ball_dist_x = 20;	//这个值越大 拿球越平滑
double  x = 20;
/*
GetBall::GetBall()
{	
	WorldModel worldModel;
	isSimulation = model->get_simulation();
	isSimulation ? get_ball_buf = -4 : get_ball_buf = 5;
	isSimulation ? away_ball_dist_x = 20 : away_ball_dist_x = 40;
}

GetBall::~GetBall()
{
}*/

void get_ball(const WorldModel* model){

	// 判断/获得是否处于 模拟状态
	isSimulation = model->get_simulation();		// isSimulation 模拟

	// A ? B : C   三目运算符，当条件 A 成立，则返回 B 语句，否则返回 C 语句
	// 若是，令get_ball_buf为-4, away_ball_dist_x 为20。
	// 若不是，令为 5 和 40。
	isSimulation ? get_ball_buf = -4 : get_ball_buf = 5;
	isSimulation ? away_ball_dist_x = 20 : away_ball_dist_x = 40;
}


//判断朝向对方球门角度范围，如果大于-90度小于90度，返回true
bool toward_opp_goal(float dir){
	return (dir < PI / 2 && dir > -PI / 2);
}

float ball_x_angle(const WorldModel* model){

	// vector<point2f> 以二维（浮点）坐标点为类型的 vector向量   
	// 一个存储球二维浮点型坐标点的数组
	vector<point2f> ball_points;
	ball_points.resize(8);	// 数组的大小改为8个元素

	// 迭代8次，将8次获取到的球的坐标存入 ball_points 向量
	for (int i = 0; i < 8; i++){
		const point2f& temp_points = model->get_ball_pos(i);
		ball_points.push_back(temp_points);
	}
	// TODO: 未知返回值
	return Maths::least_squares(ball_points);
}



/*

在 player_plan 中，究竟通过WorldModel* model 获得了多少信息

1、小球当前图像帧坐标位置				ball 
2、小球当前帧的上一帧图像坐标信息		last_ball
3、我方receiver球员坐标位置信息		receive_ball_player
4、我方robot_id小车坐标信息			get_ball_player
5、敌方球门中点			opp_goal
6、我方receier_id小车朝向信息， rece_dir
7、获得以receive_ball_player为原点的极坐标，ROBOY_HEAD为极坐标length,rece_dir为极坐标angle		
   rece_head_pos = receive_ball_player + Maths::vector2polar(ROBOT_HEAD, rece_dir);
8、获得我方robot_id小车朝向信息	dir
9、获得receive_ball_player到ball向量的角度，注意：所有角度计算为向量与场地x轴正方向逆时针夹角
   receive2ball
10、获得对方球门到球的向量角度  opp_goal2ball
11、获得ball到对方球门的向量角度	ball2opp_goal
12、获得对方球门到球的向量长度	ball_away_goal
13、获得球到get_ball_player的向量长度 player_away_ball
14、获得对方球门到get_ball_player的长度 player_away_goal
15、获得球当前坐标到上一坐标位置向量的长度 ball_moving_dist
16、判断dir与对方球门的角度关系，看toward_opp_goal函数 is_toward_opp_goal
17、判断小车是否在球与对方球门之间  ball_behind_player = ball_away_goal + BALL_SIZE + MAX_ROBOT_SIZE> player_away_goal;
18、判断小球是否运动  ball_moving = (ball_moving_dist < 0.8) ? false : true;
19、
20、

	


*/



//robot_id为拿球小车车号，receiver_id为接球小车车号
PlayerTask player_plan(const WorldModel* model, int robot_id, int receiver_id){
	//创建PlayerTask对象
	PlayerTask task;
	receiver_id = robot_id;
	// 以下为执行拿球需要的参数，相关常量查看constaants.h
	// 获得小球当前图像帧坐标位置
	// 重点：小球的坐标信息都以图像帧为最小单位从视觉机接收并存储，可以把球坐标看成是一个个数组，数组索引是图像帧号，数组元素是坐标信息
	const point2f& ball = model->get_ball_pos();

	//获得小球当前帧的上一帧图像坐标信息
	const point2f& last_ball = model->get_ball_pos(1);

	//获得我方receiver_id小车坐标位置信息
	const point2f& receive_ball_player = model->get_our_player_pos(receiver_id);

	//获得我方robot_id小车坐标信息
	const point2f& get_ball_player = model->get_our_player_pos(robot_id);

	//敌方球门中点
	const point2f& opp_goal = -FieldPoint::Goal_Center_Point;

	//我方receier_id小车朝向信息，
	//注意：小车朝向为车头垂直方向与场地x轴正方向逆时针夹角
	const float rece_dir = model->get_our_player_dir(receiver_id);

	//获得以receive_ball_player为原点的极坐标，ROBOY_HEAD为极坐标length,rece_dir为极坐标angle
	const point2f& rece_head_pos = receive_ball_player + Maths::vector2polar(ROBOT_HEAD, rece_dir);

	//获得我方robot_id小车朝向信息
	const float dir = model->get_our_player_dir(robot_id);

	//获得receive_ball_player到ball向量的角度，注意：所有角度计算为向量与场地x轴正方向逆时针夹角
	float receive2ball = (ball - receive_ball_player).angle();

	//获得对方球门到球的向量角度
	float opp_goal2ball = (ball - opp_goal).angle();

	//获得ball到对方球门的向量角度
	float ball2opp_goal = (opp_goal - ball).angle();

	//获得对方球门到球的向量长度
	float ball_away_goal = (ball - opp_goal).length();

	//获得球到get_ball_player的向量长度
	float player_away_ball = (get_ball_player - ball).length();

	//获得对方球门到get_ball_player的长度
	float player_away_goal = (get_ball_player - opp_goal).length();

	//获得球当前坐标到上一坐标位置向量的长度
	float ball_moving_dist = (ball - last_ball).length();

	//判断dir与对方球门的角度关系，看toward_opp_goal函数
	bool is_toward_opp_goal = toward_opp_goal(dir);

	//判断小车是否在球与对方球门之间
	bool ball_behind_player = ball_away_goal + BALL_SIZE + MAX_ROBOT_SIZE> player_away_goal;

	//判断小球是否运动
	bool ball_moving = (ball_moving_dist < 0.8) ? false : true;

	//判断get_ball_player小车到ball向量角绝对值是否小于75度
	bool player_toward_ball = fabs((ball - get_ball_player).angle() - dir) < (PI / 2 - PI / 12) ? true : false;
	bool direct_get_ball = !ball_moving;
	bool across_ball;
	bool ball_move2target;
	float ball_moving_dir = (ball - last_ball).angle();

	// 小球在当前帧与上一帧的位移  =  球的位置 + 极坐标（球当前帧和上一帧的移动距离，移动方向）转成的二维向量坐标
	point2f ball_with_vel = ball + Maths::vector2polar(ball_moving_dist, ball_moving_dir);



	
	if (!ball_moving)	// 小球没有移动
		
		//小球的位移为当前位置
		ball_with_vel = ball;

	//  小球位移后的坐标 指向 robot_id 球员坐标 的向量的角度
	// 这个 变量 没有被用到？？？
	float ball_to_player_dir = (get_ball_player - ball_with_vel).angle();


	//球车方向(ball - get_ball_player).angle()和小车方向dir的夹角，
	//其中球车方向为小球与小车中心点的矢量方向、小车方向为垂直车头方向
	float ball_player_dir_angle = (ball - get_ball_player).angle() - dir;

	//判断小球是否在吸球嘴附近
	bool ball_beside_player_mouth = (ball - get_ball_player).length() < 25 && fabs(ball_player_dir_angle) >= 0 && fabs(ball_player_dir_angle)< PI/8;

	/*
	if (receiver_id == robot_id){	// 如果传入的两个参数相同，则朝向球门GetBall拿球

		//判断x轴方向get_ball_Player小车与球的位置关系，小车在球上侧，返回true	
		// 换一个说法： 球员比球更接近对方球门 则返回true
		bool ball_x_boundary_right = (ball.x - 2) < get_ball_player.x ? true : false;
		//判断y轴方向get_ball_player小车与球的位置关系，球在robot_id球员的左侧，返回true
		bool ball_y_boundary_right = (ball.y - 2) < get_ball_player.y ? true : false;


		//判断小球与get_ball_player车的位置关系执行拿球
			if (!ball_x_boundary_right){	// 如果球比球员更接近对方球门
					//给robot_id小车设置任务中的目标点坐标，就是让小车跑到某个点，该点以ball_with_vel为极坐标原点  
					// TODO: get_ball_buf 还是不是知道是什么，maybe 拿球缓冲距离？？
					// 拿球点为：球的位移+沿着 对方球门指向球 方向 球的半径+球员半径 + 拿球缓冲区 长度的位置
				task.target_pos = ball_with_vel + Maths::vector2polar(BALL_SIZE / 2 + MAX_ROBOT_SIZE + get_ball_buf, opp_goal2ball);
			}else{							// 球员比球更接近对方球门
				if (ball_y_boundary_right)	// 球在球员左边
					//给robot_id小车设置任务中的目标点坐标，直接设置x,y
					// x: 球的位移的x轴 - away_ball_dist_x  TODO: 原因？这个值越大，拿球越平滑
					// TODO: 当球员比球更接近对方球门时拿球的地点计算。
					task.target_pos.set(ball_with_vel.x - away_ball_dist_x, ball_with_vel.y + 35);
				else						// 球在球员右边
					task.target_pos.set(ball_with_vel.x - away_ball_dist_x, ball_with_vel.y - 35);
			}
			task.orientate = (opp_goal - ball).angle(); // 方向为 球到对方球门的向量的方向
	}
	else				// GetBall球员朝向传球球员 接球
	{   
		// TODO: 这一句和下面一句不是累赘嘛？
		//判断球与get_ball_palyer、receive_ball_player之间的位置关系，如果x轴方向球在两车下侧，得到true
		bool all_on_ball_x_boundary_left = (ball.x - 2) < get_ball_player.x && (ball.x - 2) < receive_ball_player.x;
		//如果x轴方向球在两车上侧，得到true
		bool all_on_ball_x_boundary_right = (ball.x - 2) > get_ball_player.x && (ball.x - 2) > receive_ball_player.x;

		//判断y轴方向get_ball_player小车与球的位置关系，小车在球左侧，得到true
		bool executer_onball_y_boundary_right = (ball.y - 2) < get_ball_player.y ? true : false;
		
		//判断小球与拿球车、接球车之间的位置关系执行拿球
		if (all_on_ball_x_boundary_right){	// x轴方向 球比两球员离对方球门更近
			if (executer_onball_y_boundary_right)	// y轴方向 球员在球左侧
				//设置task任务目标点坐标
				task.target_pos.set(ball_with_vel.x + away_ball_dist_x, ball_with_vel.y + 35);
			else
				task.target_pos.set(ball_with_vel.x + away_ball_dist_x, ball_with_vel.y - 35);
			
		}else if (all_on_ball_x_boundary_left){	// 两球员离球门更近
			if (executer_onball_y_boundary_right)		// 球员在球的左侧
				task.target_pos.set(ball_with_vel.x - away_ball_dist_x, ball_with_vel.y + 35);
			else
				task.target_pos.set(ball_with_vel.x - away_ball_dist_x, ball_with_vel.y - 35);

		}else{		// 球在两个球员之间
			task.target_pos = ball_with_vel + Maths::vector2polar(BALL_SIZE / 2 + MAX_ROBOT_SIZE + get_ball_buf, receive2ball);
		}
		task.orientate = (rece_head_pos - ball).angle();
	}
	*/
	task.orientate = (ball - get_ball_player).angle();
	task.target_pos = ball_with_vel + Maths::vector2polar(BALL_SIZE / 2 + MAX_ROBOT_SIZE + get_ball_buf, ball_to_player_dir);

	//判断小球在GetBall拿球车吸球嘴附近
	if (ball_beside_player_mouth){
		//执行拿球
		task.needCb = 1;
		task.target_pos = ball + Maths::vector2polar(x, anglemod(dir + PI));
	}
#if debug
	char *p1 = new char[20];
	char *p2 = new char[20];
	char *p3 = new char[20];
	char *p4 = new char[20];
	char *p5 = new char[20];
	// 以读模式打开文件
	ifstream infile;
	infile.open("wjl.dat");
	infile.getline(p1, 20, '\n');
	infile.getline(p2, 20, '\n');
	infile.getline(p3, 20, '\n');
	infile.getline(p4, 20, '\n');
	infile.getline(p5, 20, '\n');
	get_ball_buf = atof(p1);
	away_ball_dist_x = atof(p2);
	x = atof(p3);
		// 关闭打开的文件
	infile.close();
	//sscanf_s(data, "%lf$%lf$%lf", &spiral_buff, &away_ball_dist_x, &do_spiral_max_cnt);
	//// 以写模式打开文件
	ofstream outfile;
	outfile.open("log.dat", ios::app);
	outfile << "--------------------------" << endl;
	outfile << "get_ball_buf=" << get_ball_buf << endl;
	outfile << "away_ball_dist_x=" << away_ball_dist_x << endl;
	outfile << "x=" << x << endl;
	outfile << "ball_postion=" << ball << endl;
	outfile << "cat_postion=" << get_ball_player << endl;
	outfile << "cat_dir=" << dir << endl;
	outfile << "ball_get=" << ball_beside_player_mouth << endl;
	outfile << receiver_id << endl;
	outfile << robot_id << endl;
	outfile << "--------------------------" << endl;
	// 关闭打开的文件
	outfile.close();
#endif
	return task;

}




