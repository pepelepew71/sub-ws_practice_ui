var ros;
var robot_hostname;
var csvArray;

var csv = "\
25.0135076497,121.547345353,0\n\
25.0135058780,121.547395804,1\n\
25.0134527044,121.547394264,1\n\
25.0134579397,121.547296144,1\n\
25.0135054769,121.547285874,1\n\
25.0135076497,121.547345353,0";

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
    for (i = 0; i < lats.length; i++) {
        console.log(lats[i].innerHTML, lons[i].innerHTML, nums[i].value);
    }
}

function reload() {
    var result = "";
    csvArray = $.csv.toArrays(csv);
    for(i = 0; i < csvArray.length; i++) {
        result = result.concat(String.format(template, csvArray[i][0], csvArray[i][1], csvArray[i][2]));
    }
    document.getElementById('tbodyID').innerHTML = result
}

window.onload = function () {

    robot_hostname = "localhost";

    initROS();
    reload();

    window.addEventListener("beforeunload", () => shutdown());
}
