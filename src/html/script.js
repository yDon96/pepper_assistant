// Connecting to ROS
var ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});

// Subscribing to a Topic
var listener = new ROSLIB.Topic({
  ros : ros,
  name : '/listener',
  messageType : 'std_msgs/String'
});

let typed = undefined;

listener.subscribe(function(message) {
  console.log('Received message on ' + listener.name + ': ' + message.data);
  
  if(typed && typed.constructor === Typed) {
    typed.destroy();
  }

  typed = new Typed('#typed', {
    strings: [message.data],
    typeSpeed: 30,
    backSpeed: 20,
    fadeOut: true,
  });
});