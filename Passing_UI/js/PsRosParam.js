//================================================================
//speed

var speed_const = new ROSLIB.Param({
    ros: ros,
    name: '/speed_const',
});

function SetParam_Speed(num) {
    speed_const.set(num);
}

//================================================================
//yaw

var yaw_const = new ROSLIB.Param({
    ros: ros,
    name: '/yaw_const',
});

function SetParam_Yaw(num) {
    yaw_const.set(num);
}

//================================================================
//shoot

var shoot_const = new ROSLIB.Param({
    ros: ros,
    name: '/shoot_const',
});

function SetParam_Shoot(num) {
    shoot_const.set(num);
}

//================================================================
//target ball distance

var ball_distance = new ROSLIB.Param({
    ros: ros,
    name: '/target_ball_distance',
});

function SetParam_Ball_Distance(num) {
    ball_distance.set(num);
}

//================================================================
//target goal distance


var goal_distance = new ROSLIB.Param({
    ros: ros,
    name: '/target_goal_distance',
});

function SetParam_Goal_Distance(num) {
    goal_distance.set(num);
}

//================================================================
//hold ball distance


var hold_ball_distance = new ROSLIB.Param({
    ros: ros,
    name: '/hold_ball_distance',
});

function SetParam_Hold_Ball_Dis(num) {
    hold_ball_distance.set(num);
}

//================================================================
//hold ball angle


var hold_ball_angle = new ROSLIB.Param({
    ros: ros,
    name: '/hold_ball_angle',
});

function SetParam_Hold_Ball_Angle(num) {
    hold_ball_angle.set(num);
    console.log(num);
}

//================================================================
//Setting Param

function Save_Param() {
    obj = document.getElementsByName('ParameterElement');

    //obj[0] --> (double) speed_const
    SetParam_Speed(parseFloat(obj[0].value));
    //obj[1] --> (double) yaw_const
    SetParam_Yaw(parseFloat(obj[1].value));
    //obj[2] --> (double) shoot_const
    SetParam_Shoot(parseFloat(obj[2].value));
    //obj[3] --> (double) ball_distance
    SetParam_Ball_Distance(parseFloat(obj[3].value));
    //obj[4] --> (double) goal_distance
    SetParam_Goal_Distance(parseFloat(obj[4].value));
    //obj[5] --> (double) hold_ball_distance
    SetParam_Hold_Ball_Dis(parseFloat(obj[5].value));
    //obj[6] --> (double) hold_ball_angle
    SetParam_Hold_Ball_Angle(parseFloat(obj[6].value));

    PubTopic_Call_Get_Param();
    up();
}

//=================================================================
//get param

speed_const.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("ParameterElement");
        obj[0].value = value;
        console.log(value);
    } else {
        CheckGetParm = 0;
        obj[0].value = 15;
        //obj[0] = 0;
    }
});

yaw_const.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("ParameterElement");
        obj[1].value = value;
                
    } else {
        CheckGetParm = 0;
        obj[1].value = 15;
    }
});

shoot_const.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("ParameterElement");
        obj[2].value = value;
    } else {
        CheckGetParm = 0;
        obj[2].value = 50;
    }
});

ball_distance.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("ParameterElement");
        obj[3].value = value;
    } else {
        CheckGetParm = 0;
        obj[3].value = 0.6;
    }
});

goal_distance.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("ParameterElement");
        obj[4].value = value;
    } else {
        CheckGetParm = 0;
        obj[4].value = 3;
    }
});

hold_ball_distance.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("ParameterElement");
        obj[5].value = value;
    } else {
        CheckGetParm = 0;
        obj[5].value = 0.31;
    }
});

hold_ball_angle.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("ParameterElement");
        obj[6].value = value;
    } else {
        CheckGetParm = 0;
        obj[6].value = 3;
    }
});