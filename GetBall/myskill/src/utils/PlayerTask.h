#ifndef BASETASK_H
#define BASETASK_H
#include "vector.h"
#include "singleton.h"
#include <string>
#define error_robot_id 9999
#define get_ball_threshold 16
enum RobotRole
{
	NoRole,
	Kicker,
	LeftMiddle,
	RightMiddle,
	LeftBack,
	RightBack,
	GoalieRole
};
class PlayerRole
{
public:
	PlayerRole(){ role = NoRole; num_id = error_robot_id; };
	~PlayerRole(){};
	RobotRole role;
	int num_id;
};


//���������������skill��Ҫ�������ò�ͬ������skillִ�е�����̳�PlayeTask��
class PlayerTask
{
public:
	PlayerTask(){ memset(this,0,sizeof(PlayerTask)); };
	PlayerTask(const PlayerTask& task){
		memcpy(this,&task,sizeof(PlayerTask));
	}
	~PlayerTask(){};
	int flag;
	RobotRole role;                                             //��Ա��ɫ
	point2f target_pos;											// ȫ��Ŀ���
	double orientate;												// ȫ��Ŀ�굽�㳯��
	point2f global_vel;											// ȫ��Ŀ�굽��ƽ���ٶ�	
	double rot_vel;													// ȫ��Ŀ�굽��ת���ٶ�
	int rot_dir;														// ��ת�ķ���
	/// �˶����� �� ���ڵײ��˶����� ��ָ����ǩ����
	double maxAcceleration;										// �����ٶ�
	double maxDeceleration;										// �����ٶ�

	/// ������� �� ����ƽ��������� ��Ĭ��ʹ��
	bool needKick;													// ������ִ�п���
	bool isPass;													// �Ƿ���д���
	bool needCb;
	bool isChipKick;												// ������ƽ��
	double kickPrecision;											// �����򾫶�
	double kickPower;												// ��������
	double chipKickPower;											// ��������	
};

class CTaskMeditator
{
public:
	CTaskMeditator(){ 
		task_list = new PlayerTask[MaxRobotsTask]();
		is_at_point = new bool[MaxRobotsTask]();
	}
	~CTaskMeditator(){
		if (task_list) {
			delete task_list;
			task_list = NULL;
		}
		if (is_at_point){
			delete is_at_point;
			is_at_point = NULL;
		}
	};
	void set_task(int id,const PlayerTask& task){
		if (id < 0 || id>11) return;
		task_list[id] = task;
	}

	const PlayerTask& get_task(int id){
		if (id < 0 || id>11) return task_list[0];
		return task_list[id];
	}
	const bool arrive_point(int id, const point2f& current_pos, const float& dir){
		if (id < 0 || id>11) return false;
		if ((current_pos - task_list[id].target_pos).length() < arrive_point_range/*&&(fabs(anglemod(dir - task_list[id].orientate)))<5*PI/180*/)
			is_at_point[id] = true;
		else
			is_at_point[id] = false;
		return is_at_point[id];
	}
private:
	static const int MaxRobotsTask = 6;
	static const int arrive_point_range = 10;
	PlayerTask *task_list;
	bool* is_at_point;
};

typedef Singleton<CTaskMeditator> TaskMeditator;
#endif