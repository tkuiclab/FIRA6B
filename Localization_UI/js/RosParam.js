var IsSimulator = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/IsSimulator'
    // messageType: 'std_msgs/Int32'
});
var SPlanningVelocityBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SPlanning_Velocity'
});
var HoldConditionBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/hold_condition'
});
function GeneralTransfer(){
        var Box = [];
        var Box1 = [];
        Smallbox = document.getElementsByName("SPlanningVelocityElement");
        Smallbox1 = document.getElementsByName("BallElement");

        for(var i = 0 ;i < Smallbox.length ;i++){
        temp = Smallbox[i].value
        Box[i] = parseFloat(temp);
    }
       for(var i = 0;i < Smallbox1.length; i++){
        temp = Smallbox1[i].value
        Box1[i] = parseFloat(temp);

       }
    SPlanningVelocityBox.set(Box);
    HoldConditionBox.set(Box1);
    console.log(Box);
    console.log(Box1);

}
//GeneralReset
function GeneralReset(){
        var Smallbox;
        var obj = [2.2,0.3,80.0,50.0,20,3,144,5];
        var obj2 = [3.0,0.33,9.0,0.4];
        Smallbox = document.getElementsByName("SPlanningVelocityElement");
        Smallbox1 = document.getElementsByName("BallElement");

        for(var i = 0 ;i < Smallbox.length ;i++){
        Smallbox[i].value = obj[i];
        }
        for(var i=0;i<Smallbox1.length ;i++){
        Smallbox1[i].value = obj2[i];
        }
    
    
        GeneralTransfer();
        // console.log(obj);
}
//GeneralGet
var obj;
SPlanningVelocityBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("SPlanningVelocityElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
