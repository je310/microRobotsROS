var state=0;
var are_you_here=0;
var your_name="dummy";
var target_name=["kenny","hussien","tymoor","shobitha","mark"];
var current_target="";
var temp_list=[];
var highly_target="mark";
var tempINFO="";
var tempIMG="";
var willing="";
var time_left=5;
var answered_question=false;

function contains(a, obj) {
    for (var i = 0; i < a.length; i++) {
        if (a[i] === obj) {
            return true;
        }
    }
    return false;
}

function go_to_state(input_state){
	state=input_state;
	// This is the welcome state of the robot
if (state==0){
	document.getElementById("state_0").style.display="block";
	document.getElementById("state_1").style.display="none";
	document.getElementById("state_2").style.display="none";
	document.getElementById("state_3").style.display="none";
	document.getElementById("state_4").style.display="none";
  document.getElementById("state_5").style.display="none";
	document.getElementById("state_-2").style.display="none";
  document.getElementById("state_-3").style.display="none";

    

}else if (state==1){
	// this is the state that said the robot is on duty finding someone
	document.getElementById("state_0").style.display="none";
	document.getElementById("state_1").style.display="block";
	document.getElementById("state_2").style.display="none";
	document.getElementById("state_3").style.display="none";
	document.getElementById("state_4").style.display="none";
    document.getElementById("state_5").style.display="none";
    document.getElementById("state_-2").style.display="none";
  document.getElementById("state_-3").style.display="none";

}else if (state==2){
	//important state
	// this is the state that said the robot asking questions
  document.getElementById("state_0").style.display="none";
  document.getElementById("state_1").style.display="none";
  document.getElementById("state_2").style.display="block";
  document.getElementById("state_3").style.display="none";
  document.getElementById("state_4").style.display="none";
  document.getElementById("state_5").style.display="none";
  document.getElementById("state_-2").style.display="none"; 
  document.getElementById("state_-3").style.display="none";


}else if (state==3){
	//leading status
     temp_list=target_name.slice(0);
     go_to_state(-1);
}else if (state==4){
	document.getElementById("state_0").style.display="none";
	document.getElementById("state_1").style.display="none";
	document.getElementById("state_2").style.display="none";
	document.getElementById("state_3").style.display="none";
	document.getElementById("state_4").style.display="block";
  document.getElementById("state_5").style.display="none";
	document.getElementById("state_-2").style.display="none";
    document.getElementById("state_-3").style.display="none";


//end status , like sucess or fail 
}else if (state==5){
  document.getElementById("sucess").style.display="none";
  document.getElementById("fail").style.display="none";

if (willing==""){
  document.getElementById("fail").style.display="block";
}else{
  document.getElementById("sucess").style.display="block";
}

  document.getElementById("state_0").style.display="none";
  document.getElementById("state_1").style.display="none";
  document.getElementById("state_2").style.display="none";
  document.getElementById("state_3").style.display="none";
  document.getElementById("state_4").style.display="none";
  document.getElementById("state_5").style.display="block";
  document.getElementById("state_-2").style.display="none";
  document.getElementById("state_-3").style.display="none";



//end status , like sucess or fail 
}else if (state==-3){
 document.getElementById("state_0").style.display="none";
  document.getElementById("state_1").style.display="none";
  document.getElementById("state_2").style.display="none";
  document.getElementById("state_3").style.display="none";
  document.getElementById("state_4").style.display="none";
  document.getElementById("state_5").style.display="none";
  document.getElementById("state_-2").style.display="none";
 document.getElementById("state_-3").style.display="block";


}else if (state==-1){
  answered_question=false;
  var time_left=5;
	//if there is still information
	if (temp_list.length>0){
		//check if it is exist first 
		current_target=temp_list.shift();

  loadImage("helpers_image/"+current_target+".jpg","helpers_canvas")
	document.getElementById("current_target").innerHTML=current_target;
	document.getElementById("state_0").style.display="none";
	document.getElementById("state_1").style.display="none";
	document.getElementById("state_2").style.display="none";
	document.getElementById("state_3").style.display="block";
	document.getElementById("state_4").style.display="none";
  document.getElementById("state_5").style.display="none";
	document.getElementById("state_-2").style.display="none";
    document.getElementById("state_-3").style.display="none";
  count_down_function();


//end status , like sucess or fail 
} else{
	console.log('out of list');
	reply_the_checking_information_to_robot("none",-1);
	go_to_state(-2);
}
}
else if (state==-2){
	//if there is still information

	document.getElementById("state_0").style.display="none";
	document.getElementById("state_1").style.display="none";
	document.getElementById("state_2").style.display="none";
	document.getElementById("state_3").style.display="none";
	document.getElementById("state_4").style.display="none";
  document.getElementById("state_5").style.display="none";
  document.getElementById("state_-3").style.display="none";
  document.getElementById("state_-2").style.display="block";
}
}

//utility code for array
Array.prototype.remove = function() {
    var what, a = arguments, L = a.length, ax;
    while (L && this.length) {
        what = a[--L];
        while ((ax = this.indexOf(what)) !== -1) {
            this.splice(ax, 1);
        }
    }
    return this;
};
//reply back to the robot about the situation
function reply_the_checking_information_to_robot(come_back_information,answer){
var replyString='{"want_to_help":'+answer+',"name":"'+come_back_information+'"}';
console.log(replyString);
sendRobotSingleMessage(replyString);
//give a name back to the robot
}


//parse the robot information and start handling
function robot_command_handling_function(input_json){
  try {
target_name=input_json.helpers.slice(0);
tempIMG=input_json.helpee.img;
willing=input_json.willing;
loadImage("../mobile_interface/images/"+tempIMG,"helpee_canvas");
document.getElementById("helpee_name").innerHTML=input_json.helpee.name;
document.getElementById("helpee_name2").innerHTML=input_json.helpee.name;
document.getElementById("helper_name").innerHTML=input_json.willing;
document.getElementById("skill_name").innerHTML=input_json.helpee.skill;
document.getElementById("skill_name2").innerHTML=input_json.helpee.skill;
document.getElementById("skill_name3").innerHTML=input_json.helpee.skill;

go_to_state(input_json.state);
} catch(e){
console.log("it is connecting to the database and checking for new requests")
  }
if(input_json == null){
go_to_state(0);  
}


}

//triggered when your are you here state has been triggered
function are_you_here_func(answer){
//save the information here for now
answered_question=true;

if (answer==1){
are_you_here=answer;
console.log("now the window disappear");
document.getElementsByClassName("is_someone_here")[0].style.display="none";
document.getElementsByClassName("do_you_want_to")[0].style.display="block";
} else{
go_to_state(-1);
}
}

// The function used for loading the
 function loadImage(dataURL,name_of_canvas) {
        var canvas = document.getElementById(name_of_canvas);
        var context = canvas.getContext('2d');
        var current_width=500;
        // load image from data url
        var imageObj = new Image();
        imageObj.onload = function() {
        var ratio=canvas.width/imageObj.width;
        context.clearRect(0, 0, canvas.width, canvas.height);
          canvas.height=imageObj.height*ratio;
          context.drawImage(this, 0, 0,canvas.width,imageObj.height*ratio);
        };
        imageObj.src = dataURL;
      }

function can_you_help_func(answer){
//save the information here for now
reply_the_checking_information_to_robot(current_target,answer);
document.getElementsByClassName("is_someone_here")[0].style.display="block";
document.getElementsByClassName("do_you_want_to")[0].style.display="none";
if (answer==0){
		//remove from the list
		target_name.remove(current_target);	
		go_to_state(-1);

}else{
	    go_to_state(4);
}
//
}


//The ros information has started 
  var ros = new ROSLIB.Ros({
    url : "ws://" + window.location.hostname + ":9090"
  });


  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/robot_mvc/controller',
    messageType : 'std_msgs/String'
  });

  // Then we add a callback to be called every time a message is published on this topic.
  listener.subscribe(function(message) {
    console.log('Received message on ' + listener.name + ': ' + message.data);
   tempINFO=message.data;
	  console.log(message.data);
    robot_command_handling_function(JSON.parse(message.data));
    // If desired, we can unsubscribe from the topic as well.
  });


  var interface_publisher = new ROSLIB.Topic({
    ros : ros,
    name : '/robot_mvc/view',
    messageType : 'std_msgs/String'
  });


  function sendRobotSingleMessage(string_input){
  	var tempData = new ROSLIB.Message({
    data : string_input
  });
  	interface_publisher.publish(tempData);
    console.log("sent message at view")
  }

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    reply_the_checking_information_to_robot("none",9);
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

function count_down_function(){
  if(state==-1&&!answered_question){
  if (time_left>0){
  setTimeout(function(){time_left=time_left-1;
  console.log(time_left);
  count_down_function();
  }, 1000);
}else if (time_left==0){
  time_left=5;
are_you_here_func(0);
console.log("Time finished!");
}else{
    time_left=5;
  //do nothing
}
}else{
    time_left=5;
//do nothing
}

}