#ifndef BASEVISION_H
#define BASEVISION_H
#include "vector.h"
#include <map>
#include <vector>
#define ERRORNUM 9999
#define CAMERAS 2
#define MaxPlayer 6
/*������Ϣѡ���������ݽṹԭ�����£�
1��ʹ��vector���Զ�̬�Ĵ��泡�ϵ��Ӿ����ݣ���߳���Ч�ʣ������Ӿ����ݷ���
2�� ���ʹ�����飬ֻ���Ⱦ�̬����ó��ϵ��Ӿ�����ռ����磺���ӻ����ˣ�buleRobot[6]��ͬʱ�������ݺܲ����㣨�ںϴ���ɾȥ��ͬ�������ݣ�
3�� ���ʹ��map<int,Vector2dFloat> buleRobot���ԺܺõĲ������ݺͶ�̬���䣬�����޷����㴢�泡����ͬ���ŵ����ݣ���ΪkeyΨһ*/
enum Team
{
	BULE = 0,
	YELLOW = 1
};

/*�����˳��Ŷ�Ӧ��λ��*/
typedef struct RobotProperty{
	int  id;
	float orientation;
	point2f pos;
	RobotProperty(){ id = ERRORNUM; orientation = 0.0; pos.x = 0; pos.y = 0; }
	void zero(){ id = ERRORNUM; orientation = 0.0; pos.x = 0; pos.y = 0; }
}Robot;

//��¶���û��������ͣ�ԭ���ǶԳ�����Ϣ�����κμӹ�
/*FieldMSG��Ϣ�ǳ�������/�ƶ�/���λ����Ϣ vectorsʹ��resize�ȷ����ڴ� ��������*/
typedef struct FieldMSG{
	int cameraId;
	int frame;
	std::vector<point2f>  ball;
	std::vector<Robot> buleRobot;
	std::vector<Robot> yellowRobot;
	FieldMSG(){
		cameraId = ERRORNUM;
		frame = ERRORNUM;
	}
} FieldVisionMsg;

//�����������ͱ�¶������ģ��ʹ��
class MobileObject
{
public:
	MobileObject(){ v = point2f(0, 0); }
	~MobileObject(){}
	void set_vel(const point2f t){ v = t; }
	void set_vel(float vx, float vy){ v.set(vx, vy);}
	point2f vel() const { return v; }
protected:
	point2f v;//ƽ���ٶ�
};

class PlayerObject{
public:
	PlayerObject():rotVel(0){}
	PlayerObject(float rot):rotVel(rot){}
	~PlayerObject(){}
	void set_rot(float rot){ rotVel = rot; }
	float rot() const { return rotVel; }
protected:
	float rotVel;// ��ת�ٶ�
};

///Ϊ����������ģ�鷽������Ӿ����ݣ��滻��ԭʼ��������FieldVisionMsg���������Ա���ݶ���������
//������Logger���������������£���Ա����û��˽�л�ԭ��Ƶ������
class BallVision:public MobileObject
{
public:
	BallVision(){ isBallPredict = false; isExist = false; pos.x = 0.0; pos.y = 0.0; }
	~BallVision(){}
	bool isBallPredict;//��ʶ�Ƿ���Ԥ������
	bool isExist;
	point2f pos;
};

//������Ա������������,��Ա����û��˽�л�ԭ��Ƶ������
class PlayerVision:public PlayerObject,public MobileObject
{
public:
	PlayerVision(){ isRobotPredict = false;  isExist = false; cmd_rot = 0.0; cmd_v.x = 0.0; cmd_v.y = 0.0; player.zero(); }
	PlayerVision(const Robot& r, bool isPredict, bool isFound, bool valid) :player(r), isRobotPredict(isPredict), isExist(isFound){}
	~PlayerVision(){}
	bool isRobotPredict;
	bool isExist;
	Robot player;
	point2f cmd_v;
	float cmd_rot;
};

template <class item_t>
class AllocaArray{
public:
	explicit AllocaArray(){
		for (int i = 0; i < MAXSIZE; i++){
			p[i] = nullptr;
		}
		cnt = 0;
	}

	~AllocaArray(){}

	item_t* alloc(int size){
		if (cnt >= MAXSIZE) cnt = 0;
		if (p[cnt]) free();
		p[cnt++] = new item_t[size];
		return p[cnt - 1];
	}

	void free(){
		delete[]p[cnt];
	}
private:
	int cnt;
	const static int MAXSIZE = 16;//ע��MAXSIZEҪ��Logger��maxsizeҪ��Ӧ
	item_t* p[MAXSIZE];
};

typedef AllocaArray<PlayerVision> AllocPlayer;

/*�ýӿ�����չ*/
class Vision
{
public:
	Vision();
	~Vision();
	virtual void processVision(FieldVisionMsg* visionData) = 0;
protected:
	
};
#endif