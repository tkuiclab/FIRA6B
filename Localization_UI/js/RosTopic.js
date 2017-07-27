//Region
var RegionBox = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/Location',
    messageType: 'std_msgs/Float32MultiArray'
});
//Optimization
var OptimizationBox = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/Optimization',
    messageType: 'std_msgs/Int32MultiArray'
});
//GameState
var GameState = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
var SaveParam = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/SaveParam',
    messageType: 'std_msgs/Int32'
});
//cmd_vel
var cmdVel1 = new ROSLIB.Topic({
    ros: ros,
    name: '/motion/cmd_vel',
    messageType: '/geometry_msgs/Twist'
});
//TopicFunction
function TopicRegion(Num) {
    var box = new ROSLIB.Message({
        data: [Num[0], Num[1], Num[2], Num[3], Num[4], Num[5], Num[6], Num[7], Num[8], Num[9]]
    });
    RegionBox.publish(box);
}
function TopicOptimization(Num) {
    var box = new ROSLIB.Message({
        data: [Num[0], Num[1], Num[2], Num[3]]
    });
    OptimizationBox.publish(box);
}
function topicROSGameState(state) {
    console.log(state);
    var gameState = new ROSLIB.Message({
        data: state
    });
    GameState.publish(gameState);
}
function OpenSimulator(checked) {
    var temp;
    if (checked == true) {
        // temp = new ROSLIB.Message({
        //     data: 1
        // });
        // IsSimulator.publish(temp);
        temp = 1;
        IsSimulator.set(temp);
        console.log(temp);
    } else {
        // temp = new ROSLIB.Message({
        //     data: 0
        // });
        // IsSimulator.publish(temp);
        temp = 0;
        IsSimulator.set(temp);
        console.log(temp);
    }
}
function topicSaveParam(value){
    console.log(value);
    var Param = new ROSLIB.Message({
        data:value
    })
    SaveParam.publish(Param);
}
function PublishTopicCmdVel(vec3) {
    var twist = new ROSLIB.Message({
        linear: {
            x: vec3.x,
            y: vec3.y,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: vec3.z
        }
    });
    console.log(twist);
    cmdVel1.publish(twist);
}