// Connecting to ROS
  // -----------------

  var ros = new ROSLIB.Ros();
  // If there is an error on the backend, an 'error' emit will be emitted.
	ros.on('error', function(error) {
		console.log(error);
	});

  // Find out exactly when we made a connection.
  ros.on('connection', function() {
		console.log('Connection made!');
  });

	ros.on('close', function() {
		console.log('Connection closed.');
	});  

  // First, we create a Topic object with details of the topic's name and message type.
  function connect_ws(IP,Prot){
    ros.connect('ws://'+IP+':'+Prot);	
  }
