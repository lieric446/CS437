const img = document.getElementById("stream");
////alert("renderer script loaded");
////alert(img);
// ////alert("renderer script loaded");
////alert(`Stream URL: ${window?.cameraConfig?.streamUrl}`);
img.src = window.cameraConfig.streamUrl;

window.piAPI.connectValues();

window.piAPI.onValue((value) => {
  ////alert(`Received value from Pi: ${value}`);
  document.getElementById("bluetooth").textContent = value;
  value = JSON.parse(value);
  //   //alert()
  document.getElementById("direction").textContent =
    value?.data?.steering_angle;
  document.getElementById("graydscale").textContent = value?.data?.grayscale;
  document.getElementById("distance").textContent = value?.data?.distance;
  document.getElementById("temperature").textContent = value?.data?.temperature;
  //   document.getElementById("speed").textContent = value?.data?.power;
});
