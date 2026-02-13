#ifndef MANAGER_H
#define MANAGER_H

const char manager_html[] PROGMEM = R"rawliteral(

<!DOCTYPE HTML>
<html>
<head>
  <title>Grow Buddy WiFi Konfigurator</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html, body {
      height: 100%;
      margin: 0;
      font-family: Arial, sans-serif;
      background: linear-gradient(to bottom, #001f3f, #003f7f);
      color: #ffffff;
    }
    body {
      display: flex;
      justify-content: center;
      align-items: flex-start;
      padding-top: 20px;
    }
    .container {
      max-width: 600px;
      width: 100%;
      min-height: 80vh; /* Set your preferred minimum height */
      background-color: #1e1e1e;
      padding: 20px;
      border-radius: 8px;
      box-shadow: 0 0 10px rgba(0, 0, 0, 0.5);
      display: flex;
      flex-direction: column;
      transition: height 0.3s ease-in-out;
      overflow-y: auto; /* Add scroll bar when content overflows */
    }
    h1 {
      text-align: center;
      color: #ffffff;
    }
    button {
      background-color: #007bff;
      color: #fff;
      border: none;
      padding: 10px 20px;
      cursor: pointer;
      border-radius: 4px;
      margin-bottom: 10px;
    }
    button:hover {
      background-color: #0056b3;
    }
    .network-list {
      list-style-type: none;
      padding: 0;
      flex-grow: 1;
    }
    .network-list li {
      margin-bottom: 10px;
      padding: 10px;
      background-color: #2a2a2a;
      border-radius: 4px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      cursor: pointer;
    }
    .network-list li:hover {
      background-color: #444444;
    }
    .network-info {
      flex-grow: 1;
      word-wrap: break-word;
    }
    .signal-container {
      display: flex;
      align-items: center;
    }
    .signal-strength {
      width: 50px;
      height: 10px;
      background-color: #ccc;
      border-radius: 4px;
      overflow: hidden;
      margin-left: 10px;
    }
    .signal-bar {
      height: 100%;
      background-color: #4CAF50;
      transition: width 0.3s ease;
    }
    .messages {
      margin-top: 20px;
    }
    .error {
      color: red;
    }
    .success {
      color: green;
    }
    .ssid {
      display: inline-block;
      word-wrap: break-word;
    }
    .list-header {
      display: flex;
      justify-content: space-between;
      padding: 10px;
      background-color: #333;
      border-radius: 4px;
      margin-bottom: 10px;
    }
    #connection-status {
      margin-top: 20px;
    }
  </style>
</head>
<body>
  <div class="container" id="container">
    <h1>Grow Buddy WiFi Konfigurator</h1>

    <!-- Connection status and IP address display -->
    <div id="connection-status"></div>

    <!-- Scan networks button and network list -->
    <button onclick="scanNetworks()">Isci omrezja</button>
    <div class="list-header">
      <div>Ime omrezja</div>
      <div>Jakost omrezja</div>
    </div>
    <ul id="networks" class="network-list"></ul>

    <!-- Error and success messages display -->
    <div id="messages" class="messages"></div>
  </div>

  <script>
    // Function to start scanning for networks
    function scanNetworks() {
      var xhttp = new XMLHttpRequest();
      document.getElementById("networks").innerHTML = "Iskanje omrezij...";

      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          setTimeout(function() {
            checkScanResults(true);
          }, 30000);  // 30 seconds timeout

          setTimeout(checkScanResults, 4000);
        } else if (this.readyState == 4) {
          showAlert("Failed to initiate scan. Please try again.", "error");
        }
      };

      xhttp.open("GET", "/scan", true);
      xhttp.send();
    }

    // Function to clear the network list
    function clearNetworks() {
      document.getElementById("networks").innerHTML = "";
    }

    // Function to periodically check scan results
    function checkScanResults(refresh = false) {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4) {
          if (this.status == 200) {
            if (this.responseText == "Scan running...") {
              if (refresh) {
                setTimeout(checkScanResults, 1000);  // wait 1 second and try again
              }
            } else {
              var networks = JSON.parse(this.responseText);
              if (networks.length > 0) {
                showNetworkList(networks);
              } else {
                showAlert("No networks found.", "error");
              }
            }
          } else {
            if (refresh) {
              refreshNetworks();
            }
          }
        }
      };
      xhttp.open("GET", "/scanResults", true);
      xhttp.send();
    }

    // Function to refresh network list
    function refreshNetworks() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          var networks = JSON.parse(this.responseText);
          if (networks.length > 0) {
            showNetworkList(networks);
          } else {
            showAlert("No networks found.", "error");
          }
        }
      };
      xhttp.open("GET", "/scanResults", true);
      xhttp.send();
    }

    // Function to display the network list
    function showNetworkList(networks) {
      var container = document.getElementById('container');
      var listHeight = networks.length * 50 + 100; // Approximate height per network item + header and padding
      var minHeight = parseFloat(window.getComputedStyle(container).minHeight);
      var newHeight = Math.max(listHeight, minHeight);
      container.style.minHeight = newHeight + 'px'; // Set min-height to ensure container expands

      var html = '';
      networks.forEach(function(network) {
        html += '<li onclick="selectNetwork(\'' + network.ssid + '\')">';
        html += '<div class="network-info">';
        html += '<span class="ssid">' + (network.ssid.length > 15 ? network.ssid.slice(0, 15) + '<br>' + network.ssid.slice(15) : network.ssid) + '</span>';
        html += '</div>';
        html += '<div class="signal-container">';
        html += '<span>' + Math.round(calculateSignalStrength(network.rssi)) + '%</span>';
        html += '<div class="signal-strength">';
        html += '<div class="signal-bar" style="width: ' + Math.round(calculateSignalStrength(network.rssi)) + '%;"></div>';
        html += '</div>';
        html += '</div>';
        html += '</li>';
      });
      document.getElementById("networks").innerHTML = html;
      showAlert("Iskanje zakljuceno.", "success");

      // After updating content, ensure container scrolls to top
      container.scrollTop = 0;
    }

    // Function to handle network selection and connection
function selectNetwork(ssid) {
  var password = prompt("Vnesi geslo za " + ssid + ":");
  if (password != null) {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4) {
        if (this.status == 200) {
          showAlert("Povezujem z " + ssid + "...");
          // No need to show "Connected" status here, wait for actual connection response
        } else if (this.status == 400) {
          showAlert("Incorrect password for " + ssid, "error");
        } else {
          showAlert("Failed to connect", "error");
        }
      }
    };
    xhttp.open("POST", "/connect", true);
    xhttp.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
    xhttp.send("ssid=" + encodeURIComponent(ssid) + "&password=" + encodeURIComponent(password));

    // After sending the request, you can periodically check for connection status
    checkConnectionStatus(ssid);
  }
}
    // Function to update the connection status on the webpage
    function updateConnectionStatus(ssid) {
      var statusDiv = document.getElementById("connection-status");
      statusDiv.innerHTML = "<p>Status: <strong>" + ssid + "</strong></p><p>IP naslov: <span id='ip-address'></span></p>";
      getIPAddress(); // Call function to retrieve and display IP address
    }

    // Function to clear the connection status on the webpage
    function clearConnectionStatus() {
      var statusDiv = document.getElementById("connection-status");
      statusDiv.innerHTML = "";
    }

    // Function to retrieve and display the IP address
    function getIPAddress() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          document.getElementById("ip-address").textContent = this.responseText.trim();
        }
      };
      xhttp.open("GET", "/ip", true);
      xhttp.send();
    }

    // Function to display alert messages
    function showAlert(message, type) {
      var div = document.createElement("div");
      div.textContent = message;
      div.className = type;
      document.getElementById("messages").appendChild(div);
    }

    // Function to calculate signal strength percentage
    function calculateSignalStrength(rssi) {
      var minRSSI = -100;
      var maxRSSI = -50;
      var percentage = (rssi - minRSSI) / (maxRSSI - minRSSI) * 100;
      return Math.min(100, Math.max(0, percentage));
    }

    // Initial check for connection status when the page loads
    window.onload = function() {
      checkConnectionStatus();
    };

    // Function to periodically check connection status
    function checkConnectionStatus() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          if (this.responseText.trim() !== "Ni povezave") {
            var ssid = this.responseText.trim();
            updateConnectionStatus(ssid);
          } else {
            clearConnectionStatus();
          }
        }
      };
      xhttp.open("GET", "/status", true);
      xhttp.send();
      setTimeout(checkConnectionStatus, 5000); // Check every 5 seconds
    }
  </script>
</body>
</html>

)rawliteral";

#endif // INDEX_H
