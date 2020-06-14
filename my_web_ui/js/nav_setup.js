var ros;
var robot_hostname;

function initROS() {

    ros = new ROSLIB.Ros({
        url: "ws://" + robot_hostname + ":9090"
    });

}

function shutdown() {
    ros.close();
}

window.onload = function () {

    robot_hostname = "localhost";

    initROS();

    window.addEventListener("beforeunload", () => shutdown());
}

function delete_row(btndel) {
    if (typeof(btndel) == "object") {
        $(btndel).closest("tr").remove();
    } else {
        return false;
    }
}