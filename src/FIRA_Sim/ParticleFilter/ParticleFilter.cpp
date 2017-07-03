#include "ParticleFilter.hpp"


double rate;
Particle tp;
T_Robot tr;
goal_pos tg;


ParticleFilter::ParticleFilter(){
    mSensorModel = LikeliHood;
    randomize();
}

void ParticleFilter::setMap(double width,double height,int inResolution,bool* inMap){
    mapH = height;
    mapW = width;
    resolution = inResolution;
    map = inMap;

    mapW_grid = width*resolution;
    mapH_grid = height*resolution;

}


//generate pNum particle
//input-
//  inNum: user want particle number
//class output-
//  pNum : eaqual to inNum
//  pAry : random assign position
void ParticleFilter::initParticle(int inNum){

    pNum = inNum;
    pAry.clear();
    //gen pNum particle
    for(int i= 0;i < pNum;i++){
        tp.pos(0) = randomX();
        tp.pos(1) = randomY();
        double tmp_yaw = rand()%360;
        tp.yaw = tmp_yaw/(2*M_PI);

        pAry.push_back(tp);
    }


    //init grade array
    //pGradeAry = new int[pNum];


}


//move particle
//input-
//  move: paricle move transition
void ParticleFilter::moveParticle(Vector2d move){

    Vector2d noise;

    for(int i= 0;i < pAry.size();i++){

        noise(0) = rand()%2;
        noise(1) = rand()%2;
        pAry[i].pos = pAry[i].pos + move + noise;
        if(pAry[i].pos(0) <0 || pAry[i].pos(0) >=mapW
         || pAry[i].pos(1) <0 || pAry[i].pos(1) >=mapH){
            pAry[i].pos(0) = randomX();
            pAry[i].pos(1) = randomY();
        }
    }
}

void ParticleFilter::rateGrade(std::vector<Vector2i> sensorWall_relPose){
    if(sensorWall_relPose.size() !=sensorLineNum){
        std::cout << "ParticleFilter::rateGrade() say different sensorLineNum!!" << std::endl;
        return ;
    }

    tg.pos(0) = 796;
    tg.pos(1) = 322;

    tr.pos(0) = T_robot_x;
    tr.pos(1) = T_robot_y;

    Vector2i tPos;
    for(int i= 0;i < pAry.size();i++){
        int sumGrade = 0;

        Vector2i grid_pos(pAry[i].pos(0)*resolution,pAry[i].pos(1)*resolution);
        pDist[i] = sqrt((tg.pos(0) - pAry[i].pos(0))*(tg.pos(0) - pAry[i].pos(0)) + (tg.pos(1) - pAry[i].pos(1))*(tg.pos(1) - pAry[i].pos(1)));
        tr.R_door_dist = sqrt((tr.pos(0) - tg.pos(0)) * (tr.pos(0) - tg.pos(0)) + (tr.pos(1) - tg.pos(1)) * (tr.pos(1) - tg.pos(1)));

        if(tr.R_door_dist > pDist[i]){
            if((tr.R_door_dist-pDist[i])<10)
                rate[i] = 1;
            else if((tr.R_door_dist-pDist[i])>200){
                rate[i] = 0;
            }else{
                rate[i] = pDist[i]/tr.R_door_dist*0.5;
            }
        }else{
            if((pDist[i]-tr.R_door_dist)<10)
                rate[i] = 1;
            else if((pDist[i]-tr.R_door_dist)>200){
                rate[i] = 0;
            }else{
                rate[i] = tr.R_door_dist/pDist[i]*0.5;
            }
        }

        for(int j=0;j < sensorWall_relPose.size();j++){
            //pAry[i] = pAry[i] + sensorWall_relPose[j];
            tPos= grid_pos + sensorWall_relPose[j];
            if(tPos(0) >= 0 && tPos(0) < mapW_grid &&
               tPos(1) >= 0 && tPos(1) < mapH_grid){
                int tInt = tPos(1)*mapW_grid + tPos(0);
                sumGrade+= likeliHoodMap[tPos(1)*mapW_grid + tPos(0)];
            }

        }
        pAry[i].sumGrade = sumGrade*rate[i];

    }
}

bool sort_particle(Particle a, Particle b){
    return a.sumGrade > b.sumGrade;
}

Vector2d ParticleFilter::prediction(){
    //resort all particles
    sort(pAry.begin(),pAry.end(), sort_particle);
    int allPGardeSum = 0;
    for(int i= 0;i < pAry.size()/2;i++){
        allPGardeSum+= pAry[i].sumGrade;
    }

    double xSum = 0;
    double ySum = 0;
    for(int i = 0;i < pAry.size()/2;i++){
        double tmpProbality = (double)pAry[i].sumGrade / (double)allPGardeSum;
        //std::cout << "tmpProbality=" << tmpProbality << std::endl ;
        xSum += (double)pAry[i].pos(0)*(tmpProbality);
        ySum += (double)pAry[i].pos(1)*(tmpProbality);
    }
    predictionPos(0) = xSum ;
    predictionPos(1) = ySum ;
}

void ParticleFilter::rePickParticle(){

    std::vector<Particle,  Eigen::aligned_allocator<Particle> > newAry;
    for(int i = 0;i < pAry.size();i++){
        int repickIndex = random(pAry.size()/4);      //repick from front 25%

        newAry.push_back(pAry[repickIndex]);

    }

    pAry = newAry;
}
