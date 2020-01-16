#include "myskill.h"
#include "utils/maths.h"
//用户注意；接口需要如下声明
extern "C"_declspec(dllexport) PlayerTask player_plan(const WorldModel* model, int robot_id);

enum ball_near  //PenaltyArea
{

	outOfOrbit,
	onOrbit,
	shoot
	/*RightArc_behind,
	RightArc_front,
	MiddleRectangle_behind,
	MiddleRectangle_front,
	LeftArc_behind,
	LeftArc_front,*/
};


PlayerTask player_plan(const WorldModel* model, int robot_id){
	PlayerTask task;
	bool isPasstoReceiver = true;//true->pass false->shoot
	int receiver;
	for (int i = 0; i < 6; i++){
		if (i == robot_id || i == model->get_our_goalie())
			continue;
		if (model->get_our_exist_id()[i])
			receiver = i;
	}
	const point2f& receiver_pos = model->get_our_player_pos(receiver);
	point2f& opp_goal = -FieldPoint::Goal_Center_Point;
	if (isPasstoReceiver)
		opp_goal = receiver_pos;

	const float pi = 3.1415926;
	const float& circleR = 35;
	const float& DetAngle = 0.6;
	const point2f& goal = FieldPoint::Goal_Center_Point;
	const point2f& ball = model->get_ball_pos();
	const point2f& kicker = model->get_our_player_pos(robot_id);
	const float& dir = model->get_our_player_dir(robot_id);
	ball_near orbit;
	point2f shootPosOnOrbit = ball + Maths::vector2polar(circleR,(ball-opp_goal).angle());
	float toShootDir = fabs((kicker - ball).angle() - (ball - opp_goal).angle());   //(kicker - shootPosOnOrbit).length();
	float toBallDist = (kicker - ball).length();
	float toOppGoalDir = (opp_goal - kicker).angle();
	float toBallDir = (ball - kicker).angle();
	point2f robotBallAcrossCirclePoint = ball + Maths::vector2polar(circleR, (kicker - ball).angle());
	point2f AntishootPosOnOrbit = ball + Maths::vector2polar(circleR, (opp_goal - ball).angle());
	point2f BallToRobot = kicker - ball;
	

	if (toBallDist >circleR + 10)
		orbit = outOfOrbit;
	else if (toShootDir > 1)
		orbit = onOrbit;
	else
		orbit = shoot;
	
	bool getBall = toBallDist < 10;
	float diffdir_onorbit = 0;
	float b2r = BallToRobot.angle();
	float o2b = (ball - opp_goal).angle();
	bool add;
	switch (orbit)
	{
		case outOfOrbit:
			task.target_pos = robotBallAcrossCirclePoint;
			task.orientate = toOppGoalDir;
			break;
		case onOrbit: 
			if (b2r * o2b >0){
				if (b2r > 0){
					if (b2r > o2b)
						add = false;
					else
						add = true;
				}
				else{
					if (b2r > o2b)
						add = false;
					else
						add = true;
				}
			}
			else{
				if (b2r > 0)
					add = true;
				else
					add = false;
			}
			if (add)
			{	
				//+
				task.target_pos = ball + Maths::vector2polar(circleR, BallToRobot.angle() + DetAngle);
				task.orientate = toOppGoalDir;
			}
			else
			{
				//-
				task.target_pos = ball + Maths::vector2polar(circleR, BallToRobot.angle() - DetAngle);
				task.orientate = toOppGoalDir;
			}
			break;
		case shoot: 
			task.target_pos = ball + Maths::vector2polar(5, (ball - opp_goal).angle());
			task.orientate = toOppGoalDir;
			task.needKick = true;
			task.flag = 1;
				
			if (toBallDist < 10 && fabs(model->get_our_player_dir(robot_id) - task.orientate) < 0.15){
				if (isPasstoReceiver)
					task.kickPower = 80;
				else
					task.kickPower = 127;
			}
				
			break;
	}


	return task;
}
