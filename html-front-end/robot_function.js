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
  // Create a connection to the rosbridge WebSocket server.
  ros.connect("ws://" + window.location.hostname + ":9090");
  // Publishing a Topic
  // ------------------
  // First, we create a Topic object with details of the topic's name and message type.
  var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/robot0/twist',
    messageType : 'geometry_msgs/Twist'
  });
  // Then we create the payload to be published. The object we pass in to ros.Message matches the
  // fields defined in the geometry_msgs/Twist.msg definition.
  var twist = new ROSLIB.Message({
    linear : {
      x : 0,
      y : 0,
      z : 0
    },
    angular : {
      x : 0,
      y : 0,
      z : 0
    }
  });
  // And finally, publish.
  cmdVel.publish(twist);
  //Subscribing to a Topic
  //----------------------
  // Like when publishing a topic, we first create a Topic object with details of the topic's name
  // and message type. Note that we can call publish or subscribe on the same topic object.
  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/listener',
    messageType : 'std_msgs/String'
  });
  // Then we add a callback to be called every time a message is published on this topic.

  // Calling a service
  // -----------------
  // First, we create a Service client with details of the service's name and service type.

  // Then we create a Service Request. The object we pass in to ROSLIB.ServiceRequest matches the
  // fields defined in the rospy_tutorials AddTwoInts.srv file.
  var request = new ROSLIB.ServiceRequest({
    a : 1,
    b : 2
  });
  // Setting a param value
  // ---------------------
  ros.getParams(function(params) {
    console.log(params);
  });
  // First, we create a Param object with the name of the param.
  var maxVelX = new ROSLIB.Param({
    ros : ros,
    name : 'max_vel_y'
  });
  //Then we set the value of the param, which is sent to the ROS Parameter Server.
  maxVelX.set(0.8);
  maxVelX.get(function(value) {
    console.log('MAX VAL: ' + value);
  });
  // Getting a param value
  // ---------------------
  var favoriteColor = new ROSLIB.Param({
    ros : ros,
    name : 'favorite_color'
  });

  favoriteColor.set('red');
  favoriteColor.get(function(value) {
    console.log('My robot\'s favorite color is ' + value);

})

function moveX(inputX){
  var twist2 = new ROSLIB.Message({
    linear : {
      x : inputX,
      y : 0,
      z : 0
    },
    angular : {
      x : 0,
      y : 0,
      z : 0
    }
  });
  // And finally, publish.
  cmdVel.publish(twist2);
}

function rotateZ(inputZ){
    var twist = new ROSLIB.Message({
    linear : {
      x : 0,
      y : 0,
      z : 0
    },
    angular : {
      x : 0,
      y : 0,
      z : inputZ
    }
  });
  // And finally, publish.
  cmdVel.publish(twist);
}
;