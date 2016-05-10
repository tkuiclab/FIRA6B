/********

0.
    0.1.設定地圖
        ParticleFilter::setMap(int height,int width,bool* map);
    0.2.設定粒子數
        ParticleFilter::setParticleNum(int inNum)
    0.3.灑粒子
        ParticleFilter::initParticle()
    0.4.設定地圖
        ParticleFilter::runLikeliHoodGrade()
1.move
2.move exceed 30cm
3.更新粒子位置
    ParticleFilter::updateParticlePos(int x,int y)
4.感測資訊   (360/4 = 90 條=>得到90條的距離)
5.將這90條距離資訊套到每個粒子當中 並得到分數
    ParticleFilter::rateGrade(int distNum,int *distAry)
        save grade to pGradeAry
6.直接取前50%並估計位置
    ParticleFilter::prediction()
        save data to preDictionPos
7.去除前50%並再灑後50%粒子
    ParticleFilter::rePickParticle()

note:
    1.the position,mapW, mapH is double as gazebo (meter/unit)
    2.likeliHoodMap,map,sensorWallPos is grid (10cm/grid)


**********/


#include <iostream>
//#include "../common/SetGetFun.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <vector>
#include <time.h>
#include <stdio.h>
#include <algorithm>
enum SensorModel{LikeliHood=0, SIZE_OF_ENUM };
const  std::string SensorModelName[] = { "LikeliHood" };

using namespace Eigen;

struct PStruct{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vector2d pos;
    int sumGrade;
    double P_door_dist;
    double yaw;
};

typedef struct PStruct Particle;

typedef struct RStruct{
    Vector2d pos;
    double R_door_dist;
}T_Robot;

typedef struct GStruct{
    Vector2d pos;
}goal_pos;

class ParticleFilter  {

public:
    ParticleFilter();

    void setMap(double width,double height,int inResolution,bool* map);
    void setLikeliHoodMap(int *inMap){likeliHoodMap=inMap;}
    void initParticle(){    initParticle(pNum);}
    void initParticle(int inNum);
    void moveParticle(Vector2d move);
    void rateGrade(int *distAry);
    void rateGrade(std::vector<Vector2i> sensorWall_relPose);
    Vector2d prediction();
    void rePickParticle();

    void getrobotpos(double rel_robot_x,double rel_robot_y){T_robot_x = rel_robot_x;T_robot_y = rel_robot_y;}


    void setParticleNum(int inNum) {pNum = inNum;}
    int  getParticleNum() {return pNum;}
    void setSensorLineNum(int inNum) {sensorLineNum = inNum;}
    SensorModel getSensorModel(){return  mSensorModel;  }

    std::vector<Vector2i> getPosAry_grid(){
        std::vector<Vector2i> posAry;
        for(int i=0;i < pAry.size();i++){
            Vector2i grid_pos(pAry[i].pos(0)*resolution,pAry[i].pos(1)*resolution);
            posAry.push_back(grid_pos);
        }
        return  posAry;
    }

    Vector2d getPredictionPos(){return predictionPos;}
    Vector2i getPredictionPos_grid(){
        Vector2i grid_pos(predictionPos(0)*resolution,predictionPos(1)*resolution);
        return grid_pos;
    }


    //test
    void echoPPos(){
        //test
        char tmpStr[255];
        for(int i= 0;i < pAry.size();i++){
            sprintf(tmpStr,"p(%02d)=(%d,%d)\n",i,pAry[i].pos(0),pAry[i].pos(1));
            std::cout<< tmpStr << std::endl ;
        }

        std::cout << "====end===" << std::endl ;;
    }

    void setCheatPos(Vector2d rpos){
        std::cout << "in setCheatPos rpos="<< rpos << std::endl ;;
        Vector2d p1(rpos(0)-0.1,rpos(1)+0.1);
        Vector2d p2(rpos(0)  ,rpos(1)+0.1);
        Vector2d p3(rpos(0)+0.1,rpos(1)+0.1);

        Vector2d p4(rpos(0)-0.1,rpos(1)  );
        Vector2d p5(rpos(0)+0.1,rpos(1)  );

        Vector2d p6(rpos(0)-0.1,rpos(1)-0.1);
        Vector2d p7(rpos(0)    ,rpos(1)-0.1);
        Vector2d p8(rpos(0)+0.1,rpos(1)-0.1);

        pAry[11].pos = p1;
        pAry[12].pos = p2;
        pAry[13].pos = p3;
        pAry[14].pos = p4;
        pAry[15].pos = p5;
        pAry[16].pos = p6;
        pAry[17].pos = p7;
        pAry[18].pos = p8;
    }

    Vector2d predictionPos;
private :
    SensorModel mSensorModel ;

    double T_robot_x;
    double T_robot_y;
    //particles
    std::vector<Particle,  Eigen::aligned_allocator<Particle> > pAry;

    int pNum;

    double pDist[500];//1000=pNum
    double rate[500];//1000=pNum


    //map
    bool* map;
    int* likeliHoodMap;
    double mapH;
    double mapW;
    int mapH_grid;
    int mapW_grid;

    int resolution;

    //gradeLineNum
    int sensorLineNum;

    //prediction


    //void sort_particle(Particle a, Particle b);

    //void runLikeliHoodGrade();

    //grade
    //int* pGradeAry;//size = pNum = pAry.size();
     //std::vector<Vector2i> pGradeAry;

    //random
    void randomize()
    {
        int i;
        time_t t;
        srand((unsigned) time(&t));
    }

    //output unit is meter
    //resolution to 1cm
    //ex:
    //  mapW = 60(m) , mapW*100 = 6000(cm)
    //  *0.01 = unit is meter
    double randomX() // return the value between 0 ~ N-1
    {
        return (double)( rand() % (int)(mapW*100) )*0.01;
    }

    //output unit is meter
    //resolution to 1cm
    //ex:
    //  mapH = 45(m) , mapH*100 = 4500(cm)
    //  *0.01 = unit is meter
    double randomY() // return the value between 0 ~ N-1
    {
        return  ( rand() % (int)(mapH*100) )*0.01;
    }

    double random(int N) // return the value between 0 ~ N-1
    {
        return  ( rand() % N);
    }
};
