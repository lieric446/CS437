// All of the Node.js APIs are available in the preload process.
// It has the same sandbox as a Chrome extension.
window.addEventListener("DOMContentLoaded", () => {
  const replaceText = (selector, text) => {
    const element = document.getElementById(selector);
    if (element) element.innerText = text;
  };

  for (const type of ["chrome", "node", "electron"]) {
    replaceText(`${type}-version`, process.versions[type]);
  }
});

const { contextBridge } = require("electron");
// ////alert("preload script loaded");
contextBridge.exposeInMainWorld("cameraConfig", {
  streamUrl: "http://192.168.1.102:8000/stream.mjpg",
});

// const { contextBridge } = require("electron");
const net = require("net");

let client = null;
let listeners = [];

contextBridge.exposeInMainWorld("piAPI", {
  connectValues: () => {
    ////alert("Connecting to Pi backend...");
    ////alert(client);
    if (client) return;

    client = net.createConnection(
      { host: "192.168.1.102", port: 65432 },
      () => {
        var input = document.getElementById("message").value || "center";
        //alert("message: " + input);
        const message = JSON.stringify({ command: input });
        //alert("Message to send: " + message);
        ////alert("Connected to Pi backend hi");
        client.write(`${message}\n`);
        //alert("Connected to Pi backend");
      },
    );

    ////alert("Client created, setting up listeners...");
    ////alert(client);

    client.on("data", (data) => {
      ////alert(`Received value from Pi: ${data}`);
      const text = data.toString();
      listeners.forEach((cb) => cb(text));
    });

    client.on("error", (err) => {
      console.error("Socket error:", err);
    });

    client.on("close", () => {
      client = null;
    });
  },

  onValue: (callback) => {
    ////alert("Registering listener for Pi values...");
    listeners.push(callback);
  },

  sendMessage: (msg) => {
    //alert(1);
    // window.piAPI.sendMessage(msg);
    // i = 0;
    // setInterval(function () {
    // get image from python server
    // client();
    var input = msg || document.getElementById("message").value || "left";
    //alert("message: " + input);
    const message = JSON.stringify({ command: input });
    // alert("Message to send: " + message);
    // alert("Connected to Pi backend hi");
    client.write(`${message}\n`);
    //alert("Connected to Pi backend");
    // }, 50);
  },
  showHideStream: (action) => {
    const img = document.getElementById("stream");
    const placeholder = document.getElementById("pics");
    if (action === "start_stream") {
      img.style.display = "block";
      placeholder.style.display = "none";
    } else {
      img.style.display = "none";
      placeholder.style.display = "block";
    }
  },
});

//alert("Preload script executed, APIs exposed to renderer");
// update data for every 50ms
// function
