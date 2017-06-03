const char GUIHTML[] PROGMEM = R"=====(

<!DOCTYPE HTML>
<html>
  <head>
    <title>Madox Speedo</title>
    <meta name="viewport"
          content="width=device-width, initial-scale=1,
                   maximum-scale=1, user-scalable=0"/>
    <meta name="mobile-web-app-capable" content="yes">
    <script type="text/javascript" src="config.js"></script>
    <script type="text/javascript">

var wsurl = "ws://" + window.location.hostname + ":81/";

function init(){
  setup = document.getElementById("setup");
  config = document.getElementById("config");
  speedo = document.getElementById("speedo");
  image = document.getElementById("image");
  image.style.visibility = "hidden";
  config.style.visibility = "hidden";
  document.getElementById("wsurl").value = wsurl;
  document.getElementById("ssid1").value = ssid1;
  document.getElementById("pass1").value = pass1;
  document.getElementById("ssid2").value = ssid2;
  document.getElementById("pass2").value = pass2;
  document.getElementById("hostname").value = hostname;
  canvas = document.getElementById('camera');
  ctx = canvas.getContext('2d');
  img_data = ctx.createImageData(canvas.width,canvas.height);
  data = img_data.data;
  speed_scale = 1;
}

function getimage(){
  image.style.visibility = "visible";
  speedo.style.visibility = "hidden";
  ws.send('#'); //Send in the request for the next image
}

function getspeedo(){
  image.style.visibility = "hidden";
  speedo.style.visibility = "visible";
  ws.send('?'); //Restart motion
}

function connectWS(){
  wsurl = document.getElementById('wsurl').value;
  ws = new WebSocket(wsurl);
  ws.binaryType = "arraybuffer";
  ws.onopen = function(){
    setup.style.visibility = "hidden";
    
    ws.send('?'); //default start on the motion
    aliveID = setTimeout(function(){ws.close();}, 1000);
  };

  ws.onmessage = function (evt) 
  { 
    var msg = evt.data;
    
    //Clear timeout regardless of message type
    clearTimeout(aliveID);
    aliveID = setTimeout(function(){ws.close();}, 1000);
    
    if(typeof msg === "string" ) {
      //Should be a JSON motion message
      eval("MOTIONDATA=" + msg);
      writeText("debug", "Overflow : " + MOTIONDATA["overflow"] +
                         "<br>Motion : " + MOTIONDATA["motion"] +
                         "<br>dx : "     + MOTIONDATA["dx"] + 
                         "<br>dy : "     + MOTIONDATA["dy"] + 
                         "<br>x : "     + MOTIONDATA["x"] + 
                         "<br>y : "     + MOTIONDATA["y"] + 
                         "<br>surfq : "  + MOTIONDATA["surfq"] + 
                         "<br>period : " + MOTIONDATA["period"]);
      speed_kmh = speed_scale * MOTIONDATA["dy"] / (MOTIONDATA["period"]/1000000);
      speed_txt = speed_kmh.toFixed(2);
      writeText("speed", speed_txt);
      ws.send('!'); //Random meh message (This is to provide the TCP Async Ack, use this instead of modifying the library)
    } else {
      //Image data
      let bytearray = new Uint8Array(msg);
      for (var i = 0; i < data.length; i += 4) {
        data[i + 3] = 255 - bytearray[i/4]; //(alpha 255 = black, 0 = white)
      }
      ctx.putImageData(img_data, 0, 0);
      if(image.style.visibility == "visible"){
        ws.send('#'); //Send in the request for the next image
      }
    }
  };

  ws.onclose = function()
  { 
    setup.style.visibility = "visible";
  };
}

function writeText(id, text){
  document.getElementById(id).innerHTML = text;
}
    </script>
    <style type="text/css">
body {
  margin: 0px;
  background-color: #000033;
  overflow: hidden;
  font-size: 12pt;
  font-family: Arial;
  text-align: center;
  color: #FFFFFF;
  -webkit-user-select: none;
  -moz-user-select: none;
  -ms-user-select: none;
  user-select: none;
}
button, input, textarea, select, option {
  color: #FFFFFF;
  font-size: 12pt;
  font-family: Arial, Verdana, Helvetica, sans-serif;
  border: solid #003399 1px;
  background-color: #000440;
  overflow: hidden;
}
.box {
  width: 50%;
  height: 100vh;
  display: inline-block;
  box-sizing: border-box;
  border: 2px ridge #003399;
}
.centre {
  width: 100%;
  height: 100%;
  display: flex;
  justify-content: center; 
  align-items: center;
}
#setup, #config, #image, #speedo{
  width: 100%;
  height: 100%;
  background-color: #000033;
  position: absolute;
  top: 0px;
  left: 0px;
  border:none;
}
#setup {
  z-index:999;
}
#camera{
  image-rendering: pixelated;
  width:600px;
  height:600px;
  border: 2px ridge #003399;
  background: #ffffff;
  transform: scale(1, -1);
}
#config {
  z-index:9999;
}
#speed {
  font-size: 60pt;
}
    </style>
  </head>
  <body onload="init();">
    <div id="speedo">
      <div class="box" style="width: 100%">
        <div id="full" class="centre">
          <div id="speed">69</div>
          <br>
          <div id="debug">Dummy Text</div>
          <br>
          <button onclick="getimage();" style="width: 300px; height: 100px;">Get Image</button>
        </div>
      </div>
    </div>
    <div id="image">
      <div class="box" style="width: 100%">
        <div id="full" class="centre">
          <div>
            <canvas id="camera" width="30" height="30"</canvas>
          </div>
          <br>
          <button onclick="getspeedo();" style="width: 300px; height: 100px;">Get Speedo</button>
        </div>
      </div>
    </div>
    <div id="setup">
      <div class="centre">
        <div>
          <h1>MSpeedo Connection</h1>
          <input  id="wsurl" type="text" size="30" value=""/><br><br>
          <!---
          <button onclick="document.getElementById('wsurl').value = 'ws://192.168.1.130:81/';">Dummy URL</button><br>
          --->
          <button onclick="connectWS();" style="width: 300px; height: 100px;">Connect</button>
          <br><br><br>
          <button onclick="config.style.visibility = 'visible';">Config</button>
        <div>
      </div>
    </div>
    <div id="config">
      <div class="centre">
        <div>
          <h1>Optional Configuration</h1>
          <form action="configwifi" method="get">
            <h2>Wi-Fi</h2>
            HOSTNAME : <input type="text"     name="hostname" id="hostname" size="31" value=""/><br>
            SSID1    : <input type="text"     name="ssid1" id="ssid1" size="31" value=""/><br>
            PSK1     : <input type="password" name="pass1" id="pass1" size="31" value=""/><br>
            SSID2    : <input type="text"     name="ssid2" id="ssid2" size="31" value=""/><br>
            PSK2     : <input type="password" name="pass2" id="pass2" size="31" value=""/><br>
            <br>
            <input type="submit" value="Submit">
          </form>      
          <br><br>
          <button onclick="config.style.visibility = 'hidden';">Cancel</button>
        <div>
      </div>
    </div>
  </body>
</html>

)=====";
