var twist;
var manager;
var ros;
var batterySub;
var statusSub;
var gpsSub;
var cmdVelPub;
var servo1Pub, servo2Pub;
var servo1Val, servo2Val;
var servo1Last = 0, servo2Last = 0;
var twistIntervalID;
var servoIntervalID;
var robot_hostname;

var service_record_client;
var service_turn_on_client;

var vx_last = 0.0;
var wz_last = 0.0;

var max_linear_speed = 0.5;
var max_angular_speed = 1.2;

function initROS() {

    ros = new ROSLIB.Ros({
        url: "ws://" + robot_hostname + ":9090"
    });

    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });

    cmdVelPub = new ROSLIB.Topic({
        ros: ros,
        name: 'cmd_vel',
        messageType: 'geometry_msgs/Twist',
        queue_size: 10
    });
    cmdVelPub.advertise();

    servo1Pub = new ROSLIB.Topic({
        ros: ros,
        name: 'servo1/angle',
        messageType: 'std_msgs/Int16',
        latch: true,
        queue_size: 5
    });
    servo1Pub.advertise();

    servo2Pub = new ROSLIB.Topic({
        ros: ros,
        name: 'servo2/angle',
        messageType: 'std_msgs/Int16',
        latch: true,
        queue_size: 5
    });
    servo2Pub.advertise();

    systemRebootPub = new ROSLIB.Topic({
        ros: ros,
        name: 'system/reboot',
        messageType: 'std_msgs/Empty'
    });
    systemRebootPub.advertise();

    systemShutdownPub = new ROSLIB.Topic({
        ros: ros,
        name: 'system/shutdown',
        messageType: 'std_msgs/Empty'
    });
    systemShutdownPub.advertise();

    batterySub = new ROSLIB.Topic({
        ros : ros,
        name : 'battery',
        messageType : 'std_msgs/Float32',
        queue_length: 1
    });
    batterySub.subscribe(batteryCallback);

    statusSub = new ROSLIB.Topic({
        ros : ros,
        name : 'status_for_web',
        messageType : 'std_msgs/String',
        queue_length: 1
    });
    statusSub.subscribe(statusCallback);

    gpsSub = new ROSLIB.Topic({
        ros : ros,
        name : 'gps_fix',
        messageType : 'sensor_msgs/NavSatFix',
        queue_length: 1
    });
    gpsSub.subscribe(gpsCallback);

    service_record_client = new ROSLIB.Service({
        ros : ros,
        name : '/gps_rec/record',
        serviceType : 'std_srvs/Empty'
    })

    service_turn_on_client = new ROSLIB.Service({
        ros : ros,
        name : '/switch/turn_on',
        serviceType : 'std_srvs/SetBool'
    });
}

function initSliders() {

    $('#s1-slider').slider({
        tooltip: 'show',
        min: -90,
        max: 0,
        step: 1,
        value: 0
    });
    $('#s1-slider').on("slide", function(slideEvt) {
        servo1Val = slideEvt.value;
    });

    $('#s2-slider').slider({
        tooltip: 'show',
        min: -90,
        max: 0,
        step: 1,
        value: 0
    });
    $('#s2-slider').on("slide", function(slideEvt) {
        servo2Val = slideEvt.value;
    });

}

function createJoystick() {

    joystickContainer = document.getElementById('joystick');

    manager = nipplejs.create({
        zone: joystickContainer,
        position: { left: 65 + '%', top: 50 + '%' },
        mode: 'static',
        size: 200,
        color: '#ffffff',
        restJoystick: true
    });

    manager.on('move', function (evt, nipple) {

        var lin = Math.sin(nipple.angle.radian) * nipple.distance * 0.01;
        var ang = -Math.cos(nipple.angle.radian) * nipple.distance * 0.01;

        twist.linear.x = lin * max_linear_speed;
        twist.angular.z = ang * max_angular_speed;
    });

    manager.on('end', function () {
        twist.linear.x = 0
        twist.angular.z = 0
    });
}

function initTeleopKeyboard() {
    var body = document.getElementsByTagName('body')[0];
    body.addEventListener('keydown', function(e) {
        switch(e.keyCode) {
            case 37: //left
                twist.angular.z = max_angular_speed;
                break;
            case 39: //right
                twist.angular.z = -max_angular_speed;
                break;
            case 38: ///up
                twist.linear.x = max_linear_speed;
                break;
            case 40: //down
                twist.linear.x = -max_linear_speed;
        }
    });
    body.addEventListener('keyup', function(e) {
        switch(e.keyCode) {
            case 37: //left
            case 39: //right
                twist.angular.z = 0;
                break;
            case 38: ///up
            case 40: //down
                twist.linear.x = 0;
        }
    });
}

function batteryCallback(message) {
    document.getElementById('batteryID').innerHTML = 'Voltage: ' + message.data.toPrecision(4) + 'V';
}

function statusCallback(message) {
    document.getElementById('statusID').innerHTML = message.data;
}

function gpsCallback(message) {
    document.getElementById('gpsID').innerHTML = '&nbsp;&nbsp;' + message.latitude.toFixed(6) + '&nbsp;N</br>' + message.longitude.toFixed(6) + "&nbsp;E";
}

function publishTwist() {
    if (twist.linear.x == 0.0 && twist.angular.z == 0.0 && vx_last == 0.0 && wz_last == 0.0) {
        // pass
    } else {
        cmdVelPub.publish(twist);
        vx_last = twist.linear.x;
        wz_last = twist.angular.z;
    }
}

function publishServos() {
    var servoMsg;

    if (servo1Val != servo1Last) {
        servo1Last = servo1Val;
        servoMsg = new ROSLIB.Message({
            data: servo1Val
        });
        servo1Pub.publish(servoMsg);
    }

    if (servo2Val != servo2Last) {
        servo2Last = servo2Val;
        servoMsg = new ROSLIB.Message({
            data: servo2Val
        });
        servo2Pub.publish(servoMsg);
    }

}

function shutdown() {
    clearInterval(twistIntervalID);
    clearInterval(servoIntervalID);
    cmdVelPub.unadvertise();
    servo1Pub.unadvertise();
    servo2Pub.unadvertise();
    batterySub.unsubscribe();
    ros.close();
}

function record() {
    var request = new ROSLIB.ServiceRequest({});
    service_record_client.callService(request, function(response){});
}

function start() {
    var request = new ROSLIB.ServiceRequest({data: true});
    service_turn_on_client.callService(request, function(response){});
}

function stop() {
    var request = new ROSLIB.ServiceRequest({data: false});
    service_turn_on_client.callService(request, function(response){});
}

window.onload = function () {

    robot_hostname = "localhost";

    initROS();
    initSliders();
    initTeleopKeyboard();
    createJoystick();

    video = document.getElementById('video');
    video.src = "http://" + robot_hostname + ":8080/stream?topic=/camera/image_raw&type=ros_compressed";

    twistIntervalID = setInterval(() => publishTwist(), 100); // 10 hz

    servoIntervalID = setInterval(() => publishServos(), 100); // 10 hz

    window.addEventListener("beforeunload", () => shutdown());
}

window.onblur = function(){
    twist.linear.x = 0;
    twist.angular.z = 0;
    publishTwist();
}
