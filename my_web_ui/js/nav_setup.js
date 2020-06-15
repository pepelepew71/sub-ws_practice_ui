var ros;
var robot_hostname;

var service_get_client;
var service_set_client;
var service_load_client;
var service_save_client;
var service_clear_client;
var service_turn_on_client;

var csvtxt;
var csvArray;

var template = "<tr>\n\
    <td><input type=\"button\" value=\"Delete\" onclick=\"delete_row(this);\"></td>\n\
    <td name=\"latName\">{0}</td>\n\
    <td name=\"lonName\">{1}</td>\n\
    <td><form><input name=\"numName\" type=\"number\" min=\"0\" max=\"10\" step=\"1\" value=\"{2}\" required></form></td>\n\
</tr>";

if (!String.format) {
    String.format = function(format) {
        var args = Array.prototype.slice.call(arguments, 1);
        return format.replace(/{(\d+)}/g, function(match, number) {
            return typeof args[number] != 'undefined' ? args[number] : match;
        });
    };
}

function initROS() {

    ros = new ROSLIB.Ros({
        url: "ws://" + robot_hostname + ":9090"
    });

    service_get_client = new ROSLIB.Service({
        ros : ros,
        name : '/gps_rec/get',
        serviceType : 'ros_gps_nav/GetNav'
    });

    service_set_client = new ROSLIB.Service({
        ros : ros,
        name : '/gps_rec/set',
        serviceType : 'ros_gps_nav/SetNav'
    });

    service_load_client = new ROSLIB.Service({
        ros : ros,
        name : '/gps_rec/load',
        serviceType : 'std_srvs/Empty'
    });

    service_save_client = new ROSLIB.Service({
        ros : ros,
        name : '/gps_rec/save',
        serviceType : 'std_srvs/Empty'
    });

    service_clear_client = new ROSLIB.Service({
        ros : ros,
        name : '/gps_rec/clear',
        serviceType : 'std_srvs/Empty'
    });

    service_turn_on_client = new ROSLIB.Service({
        ros : ros,
        name : '/switch/turn_on',
        serviceType : 'std_srvs/SetBool'
    });
}

function shutdown() {
    ros.close();
}

function delete_row(btndel) {
    if (typeof(btndel) == "object") {
        $(btndel).closest("tr").remove();
    } else {
        return false;
    }
}

function save() {
    var lats = document.getElementsByName('latName');
    var lons = document.getElementsByName('lonName');
    var nums = document.getElementsByName('numName');

    var result = ""
    for (i = 0; i < lats.length; i++) {
        result = result.concat(String.format("{0},{1},{2}\n", lats[i].innerHTML, lons[i].innerHTML, nums[i].value));
    }

    var request = new ROSLIB.ServiceRequest({csvtxt: result});
    service_set_client.callService(request, function(response) {});

    get_csvtxt_from_client();
}

function get_csvtxt_from_client() {
    var request = new ROSLIB.ServiceRequest({});
    service_get_client.callService(request, function(response) {
        csvtxt = response.csvtxt;
    });
}

function load() {
    csvArray = $.csv.toArrays(csvtxt);
    var result = "";
    for (i = 0; i < csvArray.length; i++) {
        result = result.concat(String.format(template, csvArray[i][0], csvArray[i][1], csvArray[i][2]));
    }
    document.getElementById('tbodyID').innerHTML = result
}

function start() {
    var request = new ROSLIB.ServiceRequest({data: true});
    service_turn_on_client.callService(request, function(response) {});
}

function stop() {
    var request = new ROSLIB.ServiceRequest({data: false});
    service_turn_on_client.callService(request, function(response) {});
}

window.onload = function () {

    robot_hostname = "localhost";

    initROS();
    get_csvtxt_from_client();

    window.addEventListener("beforeunload", () => shutdown());
}
