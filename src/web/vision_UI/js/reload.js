$("input:radio").attr("checked", false);
document.getElementById("CameraButton").checked = true;
document.getElementById("HSVButton").checked = true;
topicROSParameterButton(1);
ParameterbuttonTransfer(1);
ViewButton(1);

//showValue(this.value,'CameraLabel',0);

var notHSV=0;
function HSV(value){
	//console.log(value);
	if(value!=4){
 		notHSV=value;	
	}
	if (notHSV==0){
		topicROSParameterButton(4);
		ParameterbuttonTransfer(4);
		
		topicROSCheckButton("on");
		topicColorTransfer(this.value);		
	}/*else {
		topicROSParameterButton(notHSV);
		ParameterbuttonTransfer(notHSV);
		
		topicROSCheckButton(value);
		topicColorTransfer(this.value);		
	}*/	
}
