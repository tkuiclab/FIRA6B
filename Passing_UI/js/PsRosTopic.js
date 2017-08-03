/*========================================================*/
//Level

var level = new ROSLIB.Topic({
    ros: ros,
    name: '/level',
    messageType: 'std_msgs/Int32'
});

function PubTopic_Level_Choice(num) {
    console.log('Level : ', num);
    var level_state = new ROSLIB.Message({
        data: parseInt(num)
    });
    level.publish(level_state);
}
/*========================================================*/
//mode

var mode = new ROSLIB.Topic({
    ros: ros,
    name: '/mode',
    messageType: 'std_msgs/Int32'
});

function PubTopic_Mode_Choice(num) {
    console.log('mode : ', num);
    var mode_state = new ROSLIB.Message({
        data: parseInt(num)
    });
    mode.publish(mode_state);
}

/*========================================================*/
//start

var start = new ROSLIB.Topic({
    ros: ros,
    name: '/status',
    messageType: 'std_msgs/Int32'
});

function PubTopic_Status_Choice(num) {
    console.log(num);
    if (num)
        console.log('start !!!!');
    else
        console.log('stop !!!!');
    var start_state = new ROSLIB.Message({
        data: parseInt(num)
    });
    start.publish(start_state);
}

//====================================================================
// shoot
var shoot = new ROSLIB.Topic({
    ros: ros,
    name: '/motion/shoot',
    messageType: 'std_msgs/Int32'
});

function PubTopic_Shoot(size) {
    console.log('Shoot : ', size);
    if (size >= 100) {
        size = 100;
    }
    var shoot_size = new ROSLIB.Message({
        data: parseInt(size)
    });

    shoot.publish(shoot_size);
}

//====================================================================
//call get Parameter

var call_get = new ROSLIB.Topic({
    ros: ros,
    name: '/call_get_param',
    messageType: 'std_msgs/Int32'
});

function PubTopic_Call_Get_Param() {

    console.log('Call Get');

    var get_param = new ROSLIB.Message({
        data: parseInt(1)
    });

    call_get.publish(get_param);

}

//====================================================================
//Position

var position = new ROSLIB.Topic({
    ros: ros,
    name: '/amcl_pose',
    messageType: 'geometry_msgs/PoseWithCovariance'
});

position.subscribe(function(msg) {
    document.getElementsByName('MonitorElement')[1].innerText = "( " + msg.pose.pose.position.x + ' , ' + msg.pose.pose.position.y + ' )';
});

//====================================================================
//Position angle

var position_anlge = new ROSLIB.Topic({
    ros: ros,
    name: '/imu_3d',
    messageType: 'imu_3d/inertia'
});

position_anlge.subscribe(function(msg) {
    document.getElementsByName('MonitorElement')[2].innerText = msg.yaw;
});

//====================================================================
//cmd_vel 

var cmd_vel = new ROSLIB.Topic({
    ros: ros,
    name: '/motion/cmd_vel',
    messageType: 'geometry_msgs/Twist'
});

cmd_vel.subscribe(function(msg) {
    document.getElementsByName('MonitorElement')[3].innerText = "( " + msg.linear.x + ' , ' + msg.linear.y + ' )';
    document.getElementsByName('MonitorElement')[4].innerText = msg.angular.z;
});