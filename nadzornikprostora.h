#ifndef NADZORNIKPROSTORA_H
#define NADZORNIKPROSTORA_H

const char nadzornikprostora_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width, initial-scale=1.0'>
<title>Upravitelj Datotek</title>
<style>
  body {
    font-family: 'Segoe UI', Arial, sans-serif;
    max-width: 1200px;
    margin: 0 auto;
    padding: 20px;
    background: #1e1e1e;
    color: #ffffff;
  }
  .header {
    background: #2c2c2c;
    padding: 20px;
    border-radius: 8px;
    margin-bottom: 20px;
    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
  }
  .header h1 {
    margin: 0;
    font-size: 24px;
  }
  .header p {
    margin: 5px 0 0;
    font-size: 14px;
    color: #cccccc;
  }
  .controls {
    background: #2c2c2c;
    padding: 15px;
    border-radius: 8px;
    margin-bottom: 20px;
    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
  }
  .file-list {
    background: #252526;
    border-radius: 8px;
    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
    overflow-x: auto;
  }
  .file-table {
    width: 100%;
    border-collapse: collapse;
  }
  .file-table th, .file-table td {
    padding: 12px 15px;
    text-align: left;
    border-bottom: 1px solid #3c3c3c;
  }
  .file-table th {
    background: #333333;
    color: #ffffff;
    font-weight: 600;
    position: sticky;
    top: 0;
  }
  .file-item {
    cursor: pointer;
  }
  .file-item:hover {
    background: #3c3c3c;
  }
  .file-icon {
    width: 24px;
    height: 24px;
    margin-right: 8px;
    display: inline-flex;
    align-items: center;
    justify-content: center;
    font-size: 16px;
  }
  .file-name {
    flex: 1;
    font-weight: 400;
  }
  .file-size, .file-type {
    color: #cccccc;
    font-size: 14px;
    width: 120px;
  }
  .file-actions {
    display: flex;
    gap: 8px;
    width: 250px;
    justify-content: flex-end;
  }
  .btn {
    padding: 6px 12px;
    border: none;
    border-radius: 4px;
    cursor: pointer;
    text-decoration: none;
    font-size: 14px;
    transition: background 0.2s;
  }
  .btn-primary {
    background: #0078d4;
    color: white;
  }
  .btn-primary:hover {
    background: #005a9e;
  }
  .btn-secondary {
    background: #444444;
    color: white;
  }
  .btn-secondary:hover {
    background: #555555;
  }
  .btn-danger {
    background: #d13438;
    color: white;
  }
  .btn-danger:hover {
    background: #a00e0e;
  }
  .btn-warning {
    background: #ffb900;
    color: black;
  }
  .btn-warning:hover {
    background: #cc8c00;
  }
  .btn-success {
    background: #107c10;
    color: white;
  }
  .btn-success:hover {
    background: #0b5a0b;
  }
  .breadcrumb {
    margin-bottom: 15px;
    font-size: 14px;
    padding: 0 15px;
  }
  .breadcrumb a {
    color: #4dabf7;
    text-decoration: none;
  }
  .breadcrumb a:hover {
    text-decoration: underline;
  }
  #uploadArea {
    border: 2px dashed #555555;
    border-radius: 8px;
    padding: 20px;
    text-align: center;
    margin-bottom: 15px;
    cursor: pointer;
    background: #333333;
  }
  #uploadArea:hover {
    border-color: #0078d4;
    background: #3c3c3c;
  }
  .loading {
    text-align: center;
    padding: 20px;
    color: #cccccc;
  }
  select {
    padding: 6px;
    border-radius: 4px;
    background: #333333;
    color: #ffffff;
    border: 1px solid #555555;
  }
  .storage-stats {
    margin-bottom: 15px;
    padding: 10px;
    background: #333333;
    border-radius: 4px;
  }
  .storage-stats p {
    margin: 5px 0;
    font-size: 14px;
  }
  .progress-bar {
    background: #444444;
    border-radius: 4px;
    height: 10px;
    overflow: hidden;
    margin-top: 5px;
  }
  .progress-bar-fill {
    background: #0078d4;
    height: 100%;
    transition: width 0.3s;
  }
  .export-section {
    background: #2c2c2c;
    padding: 15px;
    border-radius: 8px;
    margin-bottom: 20px;
    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
  }
  .export-section h2 {
    margin: 0 0 10px 0;
    font-size: 18px;
  }
  #exportStatus {
    margin-top: 10px;
    font-size: 14px;
    color: #4dabf7;
  }
</style>
</head>
<body>

<div class='header'>
  <h1>Nadzornik Datotek</h1>
  <p>Brskajte in upravljajte datoteke</p>
</div>

<div class='file-list'>
  <div style='margin-bottom: 10px;'>
    <label for='storageSelect'>Shramba: </label>
    <select id='storageSelect' onchange='changeStorage(this.value)'>
      <option value='fatfs' selected>Onboard FATFS</option>
      <option value='sd'>SD Kartica</option>
    </select>
  </div>
  <div class='storage-stats' id='storageStats'>
    <p>SD Kartica: <span id='sdStats'>Nalaganje...</span></p>
    <div class='progress-bar'><div id='sdProgress' class='progress-bar-fill' style='width: 0%'></div></div>
    <p>Onboard FATFS: <span id='fatfsStats'>Nalaganje...</span></p>
    <div class='progress-bar'><div id='fatfsProgress' class='progress-bar-fill' style='width: 0%'></div></div>
  </div>
  <div id='uploadArea' onclick='document.getElementById("fileInput").click()'>
    <p>Kliknite tukaj za nalaganje datotek v <span id='uploadPath'>/</span></p>
    <input type='file' id='fileInput' style='display:none' multiple>
  </div>
  <div id='uploadProgress' style='display:none; margin: 10px 0;'>
    <p><span id='uploadFileName'></span> - <span id='uploadPercent'>0</span>%</p>
    <div class='progress-bar'>
      <div id='uploadProgressBar' class='progress-bar-fill' style='width: 0%'></div>
    </div>
    <p id='uploadStatus'></p>
  </div>
  <div class='input-group'>
    <button onclick='refreshFiles()' class='btn btn-primary'>Osveži</button>
    <button onclick='goBack()' class='btn btn-secondary'>Nazaj</button>
    <button onclick='goHome()' class='btn btn-secondary'>Korenska mapa</button>
    <button onclick='promptCreateFolder()' class='btn btn-primary'>Ustvari mapo</button>
    <button onclick='document.getElementById("backgroundInput").click()' class='btn btn-primary'>Nastavi ozadje</button>
    <button onclick='exportAllData()' class='btn btn-success'>Izvozi grafe</button>
    
    <!-- OTA Section -->
    <div style='margin-top: 10px; padding: 10px; background: #2c2c2c; border-radius: 4px;'>
      <h3 style='margin: 0 0 8px 0; font-size: 14px; color: #ffffff;'>🔄 OTA Posodobitev</h3>
      <div style='display: flex; gap: 6px; align-items: center; flex-wrap: wrap; margin-bottom: 8px;'>
        <button onclick='toggleUrlInput()' class='btn btn-primary' style='padding: 4px 8px; font-size: 11px;'>URL</button>
        <button onclick='document.getElementById("firmwareFileInput").click()' class='btn btn-success' style='padding: 4px 8px; font-size: 11px;'>Naloži .bin</button>
        <button onclick='toggleFirmwareSelect()' class='btn btn-warning' style='padding: 4px 8px; font-size: 11px;'>FATFS</button>
      </div>
      
      <!-- URL Input (hidden by default) -->
      <div id='urlInputSection' style='display: none; margin-bottom: 8px; flex-wrap: wrap; gap: 6px; align-items: center;'>
        <input type='url' id='firmwareUrl' placeholder='URL...' 
               style='flex: 1; max-width: 200px; padding: 4px 6px; border-radius: 3px; border: 1px solid #555555; background: #1e1e1e; color: #ffffff; font-size: 11px;'>
        <button onclick='downloadAndOTA()' class='btn btn-primary' style='padding: 4px 8px; font-size: 11px;'>Prenesi</button>
        <button onclick='saveAsDefaultUrl()' class='btn btn-secondary' style='padding: 4px 8px; font-size: 11px;' title='Shrani kot privzeto'>⭐</button>
      </div>
      
      <!-- FATFS Select (hidden by default) -->
      <div id='firmwareSelectSection' style='display: none; margin-bottom: 8px; flex-wrap: wrap; gap: 6px; align-items: center;'>
        <select id='firmwareSelect' style='flex: 1; max-width: 200px; padding: 4px 6px; border-radius: 3px; border: 1px solid #555555; background: #1e1e1e; color: #ffffff; font-size: 11px;'>
          <option value=''>Izberi datoteko...</option>
        </select>
        <button onclick='startLocalOTA()' class='btn btn-warning' style='padding: 4px 8px; font-size: 11px;'>Posodobi</button>
      </div>
      
      <input type='file' id='firmwareFileInput' accept='.bin' style='display:none;'>
      <div id='otaStatus' style='margin-top: 6px; font-size: 11px; color: #cccccc;'></div>
    </div>
    
    <input type='file' id='backgroundInput' style='display:none' accept='image/jpeg'>
  </div>
</div>

<div class='file-list'>
  <div class='breadcrumb' id='breadcrumb'></div>
  <table class='file-table'>
    <thead>
      <tr>
        <th style='width: 50%'>Ime</th>
        <th style='width: 15%'>Velikost</th>
        <th style='width: 15%'>Tip</th>
        <th style='width: 20%'>Dejanja</th>
      </tr>
    </thead>
    <tbody id='fileList'></tbody>
  </table>
</div>

<script>
let currentPath = '/';
let currentStorage = 'fatfs';  
const DISPLAY_WIDTH = 320;
const DISPLAY_HEIGHT = 240;

// WebSocket connection for real-time updates
let ws;
let reconnectAttempts = 0;
const maxReconnectAttempts = 5;

function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  ws = new WebSocket(`${protocol}//${window.location.host}/ws`);
  
  ws.onopen = () => {
    console.log('WebSocket Connected');
    reconnectAttempts = 0;
  };
  
  ws.onclose = () => {
    console.log('WebSocket Disconnected');
    if (reconnectAttempts < maxReconnectAttempts) {
      reconnectAttempts++;
      setTimeout(connectWebSocket, 2000);
    }
  };
  
  ws.onerror = (error) => {
    console.error('WebSocket Error:', error);
  };
}

connectWebSocket();

// Default URL management
async function loadDefaultUrl() {
  try {
    const response = await fetch('/api/defaultUrl');
    const data = await response.json();
    if (data.defaultUrl && data.defaultUrl.trim() !== '') {
      document.getElementById('firmwareUrl').value = data.defaultUrl;
      console.log('Default URL loaded:', data.defaultUrl);
    }
  } catch (error) {
    console.error('Failed to load default URL:', error);
  }
}

async function saveAsDefaultUrl() {
  const urlInput = document.getElementById('firmwareUrl');
  const url = urlInput.value.trim();
  
  if (!url) {
    alert('Prosim vnesite URL za shranjevanje kot privzeti');
    return;
  }
  
  try {
    const formData = new FormData();
    formData.append('url', url);
    
    const response = await fetch('/api/defaultUrl', {
      method: 'POST',
      body: formData
    });
    
    const result = await response.json();
    
    if (result.success) {
      alert('Privzeti URL je bil uspešno shranjen!');
      console.log('Default URL saved:', url);
    } else {
      alert('Napaka pri shranjevanju privzetega URL: ' + result.message);
    }
  } catch (error) {
    console.error('Error saving default URL:', error);
    alert('Napaka pri shranjevanju privzetega URL');
  }
}

// Load default URL when page loads
window.addEventListener('load', function() {
  loadDefaultUrl();
});

// Data export functionality
async function exportAllData() {
  // Create temporary status div if it doesn't exist
  let statusDiv = document.getElementById('exportStatus');
  if (!statusDiv) {
    statusDiv = document.createElement('div');
    statusDiv.id = 'exportStatus';
    statusDiv.style.marginTop = '10px';
    statusDiv.style.fontSize = '14px';
    statusDiv.style.color = '#4dabf7';
    
    // Insert after the button
    const button = event.target;
    button.parentNode.insertBefore(statusDiv, button.nextSibling);
  }
  
  statusDiv.textContent = 'Nalaganje podatkov...';
  
  try {
    // Download all required binary files
    const files = {
      dates: {
        dnevivzorec: await downloadBinFile('/dnevivzorec.bin', 'int'),
        mesecvzorec: await downloadBinFile('/mesecvzorec.bin', 'int'),
        letovzorec: await downloadBinFile('/letovzorec.bin', 'int')
      },
      temperature: {
        tempPodat: await downloadBinFile('/tempPodat.bin', 'char2d', 365, 25),
        povprecna: await downloadBinFile('/povprecnadnevnaT.bin', 'char'),
        najvisja: await downloadBinFile('/najvisjadnevnaT.bin', 'char'),
        najnizja: await downloadBinFile('/najnizjadnevnaT.bin', 'char')
      },
      humidity: {
        RH: await downloadBinFile('/RH.bin', 'char2d', 365, 25),
        povprecna: await downloadBinFile('/povprecnadnevnaRH.bin', 'char'),
        najvisja: await downloadBinFile('/najvisjadnevnaRH.bin', 'char'),
        najnizja: await downloadBinFile('/najnizjadnevnaRH.bin', 'char')
      },
      sensor1: {
        data: await downloadBinFile('/senzor1.bin', 'char2d', 365, 25),
        najnizja: await downloadBinFile('/najnizjadnevnasens1.bin', 'char')
      },
      sensor2: {
        data: await downloadBinFile('/senzor2.bin', 'char2d', 365, 25),
        najnizja: await downloadBinFile('/najnizjadnevnasens2.bin', 'char')
      },
      sensor3: {
        data: await downloadBinFile('/senzor3.bin', 'char2d', 365, 25),
        najnizja: await downloadBinFile('/najnizjadnevnasens3.bin', 'char')
      },
      sensor4: {
        data: await downloadBinFile('/senzor4.bin', 'char2d', 365, 25),
        najnizja: await downloadBinFile('/najnizjadnevnasens4.bin', 'char')
      }
    };
    
    statusDiv.textContent = 'Validating and fixing data...';
    
    // Validate and fix all data
    validateAndFixData(files.temperature.tempPodat, -50, 60, 'tempPodat');
    validateAndFixData(files.humidity.RH, 0, 100, 'RH');
    validateAndFixData(files.sensor1.data, 0, 100, 'senzor1');
    validateAndFixData(files.sensor2.data, 0, 100, 'senzor2');
    validateAndFixData(files.sensor3.data, 0, 100, 'senzor3');
    validateAndFixData(files.sensor4.data, 0, 100, 'senzor4');
    
    // Validate summary data
    validateAndFixData(files.temperature.povprecna, -50, 60, 'povprecnadnevnaT');
    validateAndFixData(files.temperature.najvisja, -50, 60, 'najvisjadnevnaT');
    validateAndFixData(files.temperature.najnizja, -50, 60, 'najnizjadnevnaT');
    validateAndFixData(files.humidity.povprecna, 0, 100, 'povprecnadnevnaRH');
    validateAndFixData(files.humidity.najvisja, 0, 100, 'najvisjadnevnaRH');
    validateAndFixData(files.humidity.najnizja, 0, 100, 'najnizjadnevnaRH');
    validateAndFixData(files.sensor1.najnizja, 0, 100, 'najnizjadnevnasens1');
    validateAndFixData(files.sensor2.najnizja, 0, 100, 'najnizjadnevnasens2');
    validateAndFixData(files.sensor3.najnizja, 0, 100, 'najnizjadnevnasens3');
    validateAndFixData(files.sensor4.najnizja, 0, 100, 'najnizjadnevnasens4');
    
    statusDiv.textContent = 'Kreiranje HTML preglednic...';
    
    // Create separate HTML files for each dataset
    createTemperatureHTML(files.dates, files.temperature);
    createHumidityHTML(files.dates, files.humidity);
    createSensorHTML(files.dates, files.sensor1, 1);
    createSensorHTML(files.dates, files.sensor2, 2);
    createSensorHTML(files.dates, files.sensor3, 3);
    createSensorHTML(files.dates, files.sensor4, 4);
    
    statusDiv.textContent = 'Izvoz uspešen! Preneseno 6 HTML datotek.';
  } catch (error) {
    statusDiv.textContent = 'Napaka pri izvozu: ' + error.message;
    console.error('Export error:', error);
  }
}

function validateAndFixData(data, expectedMin, expectedMax, filename) {
  console.log(`[VALIDATE] Checking ${filename}: expected range ${expectedMin} to ${expectedMax}`);
  
  let fixedCount = 0;
  let invalidCount = 0;
  
  if (Array.isArray(data)) {
    // Handle 2D arrays
    for (let i = 0; i < data.length; i++) {
      if (Array.isArray(data[i])) {
        for (let j = 0; j < data[i].length; j++) {
          const val = data[i][j];
          if (val === 101) continue; // Skip sentinel values
          
          // Check for obviously corrupted data
          if (val < expectedMin || val > expectedMax || !isFinite(val)) {
            console.warn(`[FIX] Invalid value ${val} at [${i}][${j}] in ${filename} - marking as invalid`);
            data[i][j] = 101; // Mark as invalid
            invalidCount++;
          }
        }
      } else {
        // Handle 1D arrays (like Int8Array)
        const val = data[i];
        if (val === 101) continue;
        
        if (val < expectedMin || val > expectedMax || !isFinite(val)) {
          console.warn(`[FIX] Invalid value ${val} at index ${i} in ${filename} - marking as invalid`);
          data[i] = 101;
          invalidCount++;
        }
      }
    }
  } else if (data instanceof Int8Array || data instanceof Int32Array) {
    // Handle typed arrays directly
    for (let i = 0; i < data.length; i++) {
      const val = data[i];
      if (val === 101) continue;
      
      if (val < expectedMin || val > expectedMax || !isFinite(val)) {
        console.warn(`[FIX] Invalid value ${val} at index ${i} in ${filename} - marking as invalid`);
        data[i] = 101;
        invalidCount++;
      }
    }
  }
  
  if (invalidCount > 0) {
    console.log(`[VALIDATE] Fixed ${invalidCount} invalid values in ${filename}`);
  }
  
  return data;
}

async function downloadBinFile(filename, type, rows = 365, cols = 25) {
  const url = `/download?file=${encodeURIComponent(filename)}&storage=${encodeURIComponent(currentStorage)}`;
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Ni mogoče prenesti ${filename}`);
  }
  
  const arrayBuffer = await response.arrayBuffer();
  
  if (type === 'int') {
    // For int data, check if we need to handle endianness
    const int32Array = new Int32Array(arrayBuffer);
    console.log(`[DEBUG] ${filename}: First few int values:`, int32Array.slice(0, 5));
    return int32Array;
  } else if (type === 'char') {
    // For 1D char data, use Int8Array (signed 8-bit)
    const data = new Int8Array(arrayBuffer);
    console.log(`[DEBUG] ${filename}: First few char values:`, data.slice(0, 5));
    return data;
  } else if (type === 'char2d') {
    // For char data, use Int8Array (signed 8-bit)
    const data = new Int8Array(arrayBuffer);
    console.log(`[DEBUG] ${filename}: First few char values:`, data.slice(0, 10));
    const result = [];
    for (let i = 0; i < rows; i++) {
      const rowStart = i * cols;
      const rowEnd = rowStart + cols;
      const rowData = Array.from(data.slice(rowStart, rowEnd));
      
      // Convert any weird large values to proper range or mark as invalid
      const processedRow = rowData.map(val => {
        // Check for invalid data (101 is the sentinel value)
        if (val === 101) return 101;
        // Check for corrupted data (values outside expected sensor range)
        if (val < -50 || val > 150) {
          console.warn(`[WARN] Invalid sensor value ${val} at row ${i}, col ${rowData.indexOf(val)}`);
          return 101; // Mark as invalid
        }
        return val;
      });
      
      result.push(processedRow);
    }
    return result;
  }
  
  return new Int8Array(arrayBuffer);
}

function createTemperatureHTML(dates, tempData) {
  let html = `<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<title>Temperatura - Podatki</title>
<style>
  body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }
  h1 { color: #333; }
  table { border-collapse: collapse; width: 100%; background: white; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
  th { background: #0078d4; color: white; padding: 12px; text-align: left; position: sticky; top: 0; }
  td { padding: 10px; border-bottom: 1px solid #ddd; }
  tr:hover { background: #f0f0f0; }
  .invalid { color: #999; font-style: italic; }
</style>
</head>
<body>
<h1>📈 Temperatura - Dnevni podatki</h1>
<table>
<thead>
  <tr>
    <th>Datum</th>
    <th>Povprečna (°C)</th>
    <th>Najvišja (°C)</th>
    <th>Najnižja (°C)</th>`;
  
  // Add hourly columns
  for (let h = 0; h < 25; h++) {
    html += `<th>Ura ${h}</th>`;
  }
  
  html += `</tr>
</thead>
<tbody>`;
  
  // Process each day
  for (let i = 0; i < 365; i++) {
    const dan = dates.dnevivzorec[i];
    const mesec = dates.mesecvzorec[i];
    const leto = dates.letovzorec[i];
    
    // Skip invalid dates (value 101 means no data)
    if (dan === 101 || mesec === 101 || leto === 101) continue;
    
    const datum = `${dan}.${mesec}.${leto}`;
    const povp = tempData.povprecna[i] !== 101 ? tempData.povprecna[i].toFixed(1) + '°C' : '-';
    const max = tempData.najvisja[i] !== 101 ? tempData.najvisja[i].toFixed(1) + '°C' : '-';
    const min = tempData.najnizja[i] !== 101 ? tempData.najnizja[i].toFixed(1) + '°C' : '-';
    
    html += `<tr>
      <td><b>${datum}</b></td>
      <td>${povp}</td>
      <td>${max}</td>
      <td>${min}</td>`;
    
    // Add hourly data
    for (let h = 0; h < 25; h++) {
      const val = tempData.tempPodat[i][h];
      if (val === 101) {
        html += `<td class='invalid'>-</td>`;
      } else {
        // Format temperature values to 1 decimal place
        html += `<td>${val.toFixed(1)}</td>`;
      }
    }
    
    html += `</tr>`;
  }
  
  html += `</tbody>
</table>
</body>
</html>`;
  
  downloadHTML(html, 'Temperatura_podatki.html');
}

function createHumidityHTML(dates, humidData) {
  let html = `<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<title>Vlažnost - Podatki</title>
<style>
  body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }
  h1 { color: #333; }
  table { border-collapse: collapse; width: 100%; background: white; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
  th { background: #107c10; color: white; padding: 12px; text-align: left; position: sticky; top: 0; }
  td { padding: 10px; border-bottom: 1px solid #ddd; }
  tr:hover { background: #f0f0f0; }
  .invalid { color: #999; font-style: italic; }
</style>
</head>
<body>
<h1>💧 Vlažnost - Dnevni podatki</h1>
<table>
<thead>
  <tr>
    <th>Datum</th>
    <th>Povprečna (%)</th>
    <th>Najvišja (%)</th>
    <th>Najnižja (%)</th>`;
  
  for (let h = 0; h < 25; h++) {
    html += `<th>Ura ${h}</th>`;
  }
  
  html += `</tr>
</thead>
<tbody>`;
  
  for (let i = 0; i < 365; i++) {
    const dan = dates.dnevivzorec[i];
    const mesec = dates.mesecvzorec[i];
    const leto = dates.letovzorec[i];
    
    if (dan === 101 || mesec === 101 || leto === 101) continue;
    
    const datum = `${dan}.${mesec}.${leto}`;
    const povp = humidData.povprecna[i] !== 101 ? humidData.povprecna[i] + '%' : '-';
    const max = humidData.najvisja[i] !== 101 ? humidData.najvisja[i] + '%' : '-';
    const min = humidData.najnizja[i] !== 101 ? humidData.najnizja[i] + '%' : '-';
    
    html += `<tr>
      <td><b>${datum}</b></td>
      <td>${povp}</td>
      <td>${max}</td>
      <td>${min}</td>`;
    
    for (let h = 0; h < 25; h++) {
      const val = humidData.RH[i][h];
      if (val === 101) {
        html += `<td class='invalid'>-</td>`;
      } else {
        // Format humidity values as integers (percentage)
        html += `<td>${Math.round(val)}%</td>`;
      }
    }
    
    html += `</tr>`;
  }
  
  html += `</tbody>
</table>
</body>
</html>`;
  
  downloadHTML(html, 'Vlaznost_podatki.html');
}

function createSensorHTML(dates, sensorData, sensorNum) {
  let html = `<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<title>Senzor ${sensorNum} - Podatki</title>
<style>
  body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }
  h1 { color: #333; }
  table { border-collapse: collapse; width: 100%; background: white; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
  th { background: #d13438; color: white; padding: 12px; text-align: left; position: sticky; top: 0; }
  td { padding: 10px; border-bottom: 1px solid #ddd; }
  tr:hover { background: #f0f0f0; }
  .invalid { color: #999; font-style: italic; }
</style>
</head>
<body>
<h1>🌱 Senzor ${sensorNum} - Dnevni podatki</h1>
<table>
<thead>
  <tr>
    <th>Datum</th>
    <th>Najnižja vrednost</th>`;
  
  for (let h = 0; h < 25; h++) {
    html += `<th>Ura ${h}</th>`;
  }
  
  html += `</tr>
</thead>
<tbody>`;
  
  for (let i = 0; i < 365; i++) {
    const dan = dates.dnevivzorec[i];
    const mesec = dates.mesecvzorec[i];
    const leto = dates.letovzorec[i];
    
    if (dan === 101 || mesec === 101 || leto === 101) continue;
    
    const datum = `${dan}.${mesec}.${leto}`;
    const min = sensorData.najnizja[i] !== 101 ? sensorData.najnizja[i] + '%' : '-';
    
    html += `<tr>
      <td><b>${datum}</b></td>
      <td>${min}</td>`;
    
    for (let h = 0; h < 25; h++) {
      const val = sensorData.data[i][h];
      if (val === 101) {
        html += `<td class='invalid'>-</td>`;
      } else {
        // Format soil moisture values as percentages
        html += `<td>${Math.round(val)}%</td>`;
      }
    }
    
    html += `</tr>`;
  }
  
  html += `</tbody>
</table>
</body>
</html>`;
  
  downloadHTML(html, `Senzor${sensorNum}_podatki.html`);
}

function downloadHTML(content, filename) {
  const blob = new Blob([content], { type: 'text/html' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

// Original file manager functions
async function loadFiles(path, storage) {
  currentPath = path;
  currentStorage = storage;
  const fileList = document.getElementById('fileList');
  fileList.innerHTML = '<tr><td colspan="4" class="loading">Nalaganje...</td></tr>';
  try {
    const response = await fetch(`/api/list?path=${encodeURIComponent(path)}&storage=${encodeURIComponent(storage)}`);
    const files = await response.json();
    renderFileList(files);
    updateBreadcrumb();
    updateUploadPath();
  } catch (err) {
    fileList.innerHTML = `<tr><td colspan="4" class="loading">Napaka: ${err.message}</td></tr>`;
  }
}

async function loadStorageStats() {
  try {
    const response = await fetch('/api/storageStats');
    const stats = await response.json();
    
    // SD card stats
    const sdTotal = stats.sd.total / (1024 * 1024);
    const sdUsed = stats.sd.used / (1024 * 1024);
    const sdFree = stats.sd.free / (1024 * 1024);
    const sdPercent = (sdUsed / sdTotal) * 100;
    document.getElementById('sdStats').textContent = 
      `${sdUsed.toFixed(1)} MB / ${sdTotal.toFixed(1)} MB (${sdFree.toFixed(1)} MB prosto)`;
    document.getElementById('sdProgress').style.width = sdPercent + '%';
    
    // FATFS stats
    const fatTotal = stats.fatfs.total / (1024 * 1024);
    const fatUsed = stats.fatfs.used / (1024 * 1024);
    const fatFree = stats.fatfs.free / (1024 * 1024);
    const fatPercent = (fatUsed / fatTotal) * 100;
    document.getElementById('fatfsStats').textContent = 
      `${fatUsed.toFixed(1)} MB / ${fatTotal.toFixed(1)} MB (${fatFree.toFixed(1)} MB prosto)`;
    document.getElementById('fatfsProgress').style.width = fatPercent + '%';
  } catch (err) {
    console.error('Napaka pri nalaganju statistike:', err);
  }
}

function renderFileList(files) {
  const fileList = document.getElementById('fileList');
  let html = '';
  if (currentPath !== '/') {
    html += `<tr class="file-item" onclick="goBack()">
      <td><div class="file-icon">📁</div><span class="file-name">.. (Nadrejena mapa)</span></td>
      <td></td>
      <td>Mapa</td>
      <td></td>
    </tr>`;
  }
  files.forEach(file => {
    const isDir = file.type === 'directory';
    const icon = isDir ? '📁' : getFileIcon(file.name);
    const size = isDir ? '' : formatFileSize(file.size);
    const type = isDir ? 'Mapa' : getFileType(file.name);
    const action = isDir ? `loadFiles('${file.path}', '${currentStorage}')` : `window.open('/view?file=${encodeURIComponent(file.path)}&storage=${encodeURIComponent(currentStorage)}', '_blank')`;
    html += `<tr class="file-item" onclick="event.stopPropagation(); ${action}">
      <td><div class="file-icon">${icon}</div><span class="file-name">${file.name}</span></td>
      <td class="file-size">${size}</td>
      <td class="file-type">${type}</td>
      <td class="file-actions" onclick="event.stopPropagation();">
        ${isDir ? '' : `<a href="/download?file=${encodeURIComponent(file.path)}&storage=${encodeURIComponent(currentStorage)}" class="btn btn-primary" download>Prenesi</a>`}
        <button class="btn btn-danger" onclick="deleteItem('${file.path}', ${isDir})">Izbriši</button>
        <button class="btn btn-warning" onclick="promptRename('${file.path}', '${file.name}')">Preimenuj</button>
      </td>
    </tr>`;
  });
  fileList.innerHTML = html;
}

function updateBreadcrumb() {
  const breadcrumb = document.getElementById('breadcrumb');
  const parts = currentPath.split('/').filter(p => p);
  let html = `<a href="#" onclick="loadFiles('/', '${currentStorage}')">Korenska mapa</a>`;
  let pathBuild = '';
  parts.forEach(part => {
    pathBuild += '/' + part;
    html += ` / <a href="#" onclick="loadFiles('${pathBuild}', '${currentStorage}')">${part}</a>`;
  });
  breadcrumb.innerHTML = html;
}

function updateUploadPath() {
  document.getElementById('uploadPath').textContent = currentPath;
}

function getFileIcon(filename) {
  const ext = filename.toLowerCase().split('.').pop();
  const icons = {
    'txt': '📄', 'pdf': '📕', 'doc': '📘', 'docx': '📘',
    'jpg': '🖼️', 'jpeg': '🖼️', 'png': '🖼️', 'gif': '🖼️', 'bmp': '🖼️',
    'mp3': '🎵', 'wav': '🎵', 'ogg': '🎵',
    'mp4': '🎞️', 'avi': '🎞️', 'mov': '🎞️',
    'zip': '🗜️', 'rar': '🗜️', '7z': '🗜️',
    'bin': '💾', 'html': '🌐', 'htm': '🌐'
  };
  return icons[ext] || '📄';
}

function getFileType(filename) {
  const ext = filename.toLowerCase().split('.').pop();
  const types = {
    'txt': 'Besedilna datoteka',
    'pdf': 'PDF dokument',
    'doc': 'Word dokument',
    'docx': 'Word dokument',
    'jpg': 'Slika JPEG',
    'jpeg': 'Slika JPEG',
    'png': 'Slika PNG',
    'gif': 'Slika GIF',
    'bmp': 'Slika BMP',
    'mp3': 'Zvočna datoteka',
    'wav': 'Zvočna datoteka',
    'ogg': 'Zvočna datoteka',
    'mp4': 'Video datoteka',
    'avi': 'Video datoteka',
    'mov': 'Video datoteka',
    'zip': 'Stisnjena datoteka',
    'rar': 'Stisnjena datoteka',
    '7z': 'Stisnjena datoteka',
    'bin': 'Binarna datoteka',
    'html': 'HTML dokument',
    'htm': 'HTML dokument'
  };
  return types[ext] || 'Datoteka';
}

function formatFileSize(bytes) {
  if (bytes < 1024) return bytes + ' B';
  else if (bytes < 1024*1024) return (bytes/1024).toFixed(1) + ' KB';
  else return (bytes/(1024*1024)).toFixed(1) + ' MB';
}

async function promptCreateFolder() {
  const folderName = prompt('Vnesite ime nove mape:');
  if (!folderName || folderName.trim() === '') {
    alert('Ime mape je obvezno');
    return;
  }
  const folderPath = currentPath === '/' ? `/${folderName.trim()}` : `${currentPath}/${folderName.trim()}`;
  try {
    const response = await fetch('/api/createFolder', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: `path=${encodeURIComponent(folderPath)}&storage=${encodeURIComponent(currentStorage)}`
    });
    if (response.ok) {
      refreshFiles();
    } else {
      alert('Napaka pri ustvarjanju mape: ' + await response.text());
    }
  } catch (err) {
    alert('Napaka pri ustvarjanju mape: ' + err.message);
  }
}

async function deleteItem(path, isDir) {
  const itemType = isDir ? 'mapo' : 'datoteko';
  const confirmMsg = `Ste prepričani, da želite izbrisati ${itemType} ${path}?\\n\\nUporabljam močno brisanje (Force Delete) za vse datoteke.`;
  
  if (!confirm(confirmMsg)) return;
  
  try {
    const response = await fetch('/api/delete', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: `path=${encodeURIComponent(path)}&storage=${encodeURIComponent(currentStorage)}`
    });
    
    if (response.ok) {
      const result = await response.text();
      console.log('Delete result:', result);
      refreshFiles();
    } else {
      const errorMsg = await response.text();
      alert('Napaka pri brisanju: ' + errorMsg);
    }
  } catch (err) {
    alert('Napaka pri brisanju: ' + err.message);
  }
}

async function promptRename(oldPath, oldName) {
  const newName = prompt('Vnesite novo ime:', oldName);
  if (!newName || newName === oldName) return;
  const newPath = oldPath.substring(0, oldPath.lastIndexOf('/') + 1) + newName;
  try {
    const response = await fetch('/api/rename', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: `oldPath=${encodeURIComponent(oldPath)}&newPath=${encodeURIComponent(newPath)}&storage=${encodeURIComponent(currentStorage)}`
    });
    if (response.ok) {
      refreshFiles();
    } else {
      alert('Napaka pri preimenovanju: ' + await response.text());
    }
  } catch (err) {
    alert('Napaka pri preimenovanju: ' + err.message);
  }
}

function refreshFiles() {
  loadFiles(currentPath, currentStorage);
}

function goHome() {
  loadFiles('/', currentStorage);
}

function goBack() {
  if (currentPath === '/') return;
  const parentPath = currentPath.substring(0, currentPath.lastIndexOf('/')) || '/';
  loadFiles(parentPath, currentStorage);
}

function changeStorage(storage) {
  currentStorage = storage;
  loadFiles('/', currentStorage);
  document.getElementById('storageSelect').value = storage;
}

document.getElementById('fileInput').addEventListener('change', async (event) => {
  const files = event.target.files;
  if (!files.length) return;
  document.getElementById('fileList').innerHTML = '<tr><td colspan="4" class="loading">Nalaganje datotek...</td></tr>';
  
  try {
    for (let file of files) {
      await uploadFile(file);
    }
    event.target.value = '';
    refreshFiles();
  } catch (err) {
    console.error('Upload error:', err);
    alert('Napaka pri nalaganju: ' + err.message);
    event.target.value = '';
    refreshFiles();
  }
});

async function uploadFile(file) {
  const url = `/upload?path=${encodeURIComponent(currentPath)}&storage=${encodeURIComponent(currentStorage)}`;
  
  // Show progress bar
  const progressDiv = document.getElementById('uploadProgress');
  const fileNameSpan = document.getElementById('uploadFileName');
  const percentSpan = document.getElementById('uploadPercent');
  const progressBar = document.getElementById('uploadProgressBar');
  const statusSpan = document.getElementById('uploadStatus');
  
  progressDiv.style.display = 'block';
  fileNameSpan.textContent = file.name;
  percentSpan.textContent = '0';
  progressBar.style.width = '0%';
  statusSpan.textContent = 'Nalaganje...';
  
  return new Promise((resolve, reject) => {
    const xhr = new XMLHttpRequest();
    
    // Upload progress
    xhr.upload.addEventListener('progress', (e) => {
      if (e.lengthComputable) {
        const percentComplete = Math.round((e.loaded / e.total) * 100);
        percentSpan.textContent = percentComplete;
        progressBar.style.width = percentComplete + '%';
        statusSpan.textContent = `Nalaganje... ${percentComplete}%`;
      }
    });
    
    // Load complete
    xhr.addEventListener('load', () => {
      if (xhr.status >= 200 && xhr.status < 300) {
        percentSpan.textContent = '100';
        progressBar.style.width = '100%';
        statusSpan.textContent = 'Uspešno naloženo!';
        setTimeout(() => {
          progressDiv.style.display = 'none';
        }, 2000);
        resolve();
      } else {
        statusSpan.textContent = 'Napaka pri nalaganju';
        progressBar.style.width = '0%';
        setTimeout(() => {
          progressDiv.style.display = 'none';
        }, 3000);
        reject(new Error('Upload failed with status ' + xhr.status));
      }
    });
    
    // Error handling
    xhr.addEventListener('error', () => {
      statusSpan.textContent = 'Napaka pri povezovanju';
      progressBar.style.width = '0%';
      setTimeout(() => {
        progressDiv.style.display = 'none';
      }, 3000);
      reject(new Error('Network error'));
    });
    
    xhr.addEventListener('abort', () => {
      statusSpan.textContent = 'Preklicano';
      progressBar.style.width = '0%';
      setTimeout(() => {
        progressDiv.style.display = 'none';
      }, 2000);
      reject(new Error('Upload aborted'));
    });
    
    // Prepare and send request
    const formData = new FormData();
    formData.append('file', file, file.name);
    
    xhr.open('POST', url);
    xhr.send(formData);
  });
}

// Toggle URL input section
function toggleUrlInput() {
  const urlSection = document.getElementById('urlInputSection');
  const firmwareSection = document.getElementById('firmwareSelectSection');
  
  // Hide firmware select if visible
  if (firmwareSection.style.display !== 'none') {
    firmwareSection.style.display = 'none';
  }
  
  // Toggle URL input
  urlSection.style.display = urlSection.style.display === 'none' ? 'flex' : 'none';
  
  // Focus input if showing
  if (urlSection.style.display === 'flex') {
    document.getElementById('firmwareUrl').focus();
  }
}

// Toggle firmware select section
function toggleFirmwareSelect() {
  const urlSection = document.getElementById('urlInputSection');
  const firmwareSection = document.getElementById('firmwareSelectSection');
  
  // Hide URL input if visible
  if (urlSection.style.display !== 'none') {
    urlSection.style.display = 'none';
  }
  
  // Toggle firmware select
  firmwareSection.style.display = firmwareSection.style.display === 'none' ? 'flex' : 'none';
  
  // Load firmware files if showing
  if (firmwareSection.style.display === 'flex') {
    loadFirmwareFiles();
  }
}

// File input change handler for OTA
  document.getElementById('firmwareFileInput').addEventListener('change', async (event) => {
    const file = event.target.files[0];
    if (file) {
      await uploadAndStartOTA();
    }
  });

  document.getElementById('backgroundInput').addEventListener('change', async (event) => {
  const file = event.target.files[0];
  if (!file) return;
  if (!file.type.match('image/jpeg')) {
    alert('Prosimo, izberite datoteko formata JPG.');
    event.target.value = '';
    return;
  }
  document.getElementById('fileList').innerHTML = '<tr><td colspan="4" class="loading">Nastavljanje ozadja...</td></tr>';
  try {
    const resizedFile = await resizeImage(file, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    await uploadBackground(resizedFile);
    event.target.value = '';
    refreshFiles();
  } catch (err) {
    alert('Napaka pri nastavljanju ozadja: ' + err.message);
    event.target.value = '';
    refreshFiles();
  }
});

async function resizeImage(file, targetWidth, targetHeight) {
  return new Promise((resolve, reject) => {
    const img = new Image();
    img.onload = () => {
      console.log(`Original image dimensions: ${img.width}x${img.height}`);
      const canvas = document.createElement('canvas');
      canvas.width = targetWidth;
      canvas.height = targetHeight;
      const ctx = canvas.getContext('2d');
      ctx.drawImage(img, 0, 0, targetWidth, targetHeight);
      console.log(`Canvas dimensions set to: ${canvas.width}x${canvas.height}`);
      canvas.toBlob(blob => {
        if (blob) {
          console.log(`Resized image blob size: ${blob.size} bytes`);
          resolve(new File([blob], 'skyozadje.jpg', { type: 'image/jpeg' }));
        } else {
          reject(new Error('Napaka pri ustvarjanju slike'));
        }
      }, 'image/jpeg', 0.9);
    };
    img.onerror = () => reject(new Error('Napaka pri nalagu slike'));
    img.src = URL.createObjectURL(file);
  });
}

async function uploadBackground(file) {
  const url = `/api/setBackground?storage=${encodeURIComponent(currentStorage)}`;
  const formData = new FormData();
  formData.append('file', file, 'skyozadje.jpg');
  try {
    const response = await fetch(url, { method: 'POST', body: formData });
    if (!response.ok) throw new Error('Napaka pri nalaganju ozadja: ' + await response.text());
  } catch (err) {
    throw err;
  }
}

// OTA Functions
let otaWebSocket = null;
let otaReconnectTimer = null;

// Initialize OTA WebSocket
function initOTAWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws`;
  
  otaWebSocket = new WebSocket(wsUrl);
  
  otaWebSocket.onopen = function() {
    console.log('OTA WebSocket connected');
    if (otaReconnectTimer) {
      clearTimeout(otaReconnectTimer);
      otaReconnectTimer = null;
    }
  };
  
  otaWebSocket.onmessage = function(event) {
    try {
      const data = JSON.parse(event.data);
      
      if (data.ota_progress !== undefined) {
        updateOTAProgress(data.ota_progress);
      }
      
      if (data.ota_status) {
        updateOTAStatus(data.ota_status);
      }
    } catch (e) {
      console.error('Error parsing OTA WebSocket message:', e);
    }
  };
  
  otaWebSocket.onclose = function() {
    console.log('OTA WebSocket disconnected');
    // Attempt to reconnect after 3 seconds
    otaReconnectTimer = setTimeout(initOTAWebSocket, 3000);
  };
  
  otaWebSocket.onerror = function(error) {
    console.error('OTA WebSocket error:', error);
  };
}

// Update OTA status display
function updateOTAStatus(status) {
  const statusElement = document.getElementById('otaStatus');
  const statusText = {
    'ready': 'Pripravljen',
    'downloading': 'Prenašanje...',
    'updating': 'Posodabljanje...',
    'installing': 'Nameščanje...',
    'success': 'Posodobitev uspešna!',
    'failed': 'Posodobitev spodletela',
    'download_failed': 'Prenos spodletel'
  };
  
  if (statusText[status]) {
    statusElement.textContent = statusText[status];
    statusElement.style.color = status === 'success' ? '#28a745' : (status === 'failed' || status === 'download_failed' ? '#dc3545' : '#ffc107');
  }
  
  // Handle success - reload page after delay
  if (status === 'success') {
    setTimeout(() => {
      window.location.reload();
    }, 5000);
  }
}

// Download and OTA from URL
async function downloadAndOTA() {
  const urlInput = document.getElementById('firmwareUrl');
  const url = urlInput.value.trim();
  
  if (!url) {
    alert('Prosim vnesite URL za firmware');
    return;
  }
  
  if (!confirm(`Ali ste prepričani, da želite posodobiti firmware iz:\n${url}\n\nNaprava se bo ponovno zagnala.`)) {
    return;
  }
  
  try {
    const formData = new FormData();
    formData.append('url', url);
    
    const response = await fetch('/ota/download', {
      method: 'POST',
      body: formData
    });
    
    if (!response.ok) {
      throw new Error(await response.text());
    }
    
    const result = await response.text();
    console.log('OTA download started:', result);
    
  } catch (error) {
    console.error('OTA download error:', error);
    alert('Napaka pri prenosu: ' + error.message);
    updateOTAStatus('download_failed');
  }
}

// Upload firmware file and start OTA
async function uploadAndStartOTA() {
  const fileInput = document.getElementById('firmwareFileInput');
  const file = fileInput.files[0];
  
  if (!file) {
    alert('Prosim izberite .bin datoteko za nalaganje');
    return;
  }
  
  if (!file.name.endsWith('.bin')) {
    alert('Prosim izberite .bin datoteko');
    return;
  }
  
  if (!confirm(`Ali ste prepričani, da želite posodobiti firmware iz datoteke:\n${file.name} (${formatBytes(file.size)})\n\nNaprava se bo ponovno zagnala.`)) {
    return;
  }
  
  try {
    updateOTAStatus('downloading');
    
    // Use existing uploadFile function
    await uploadFile(file);
    
    // Wait a moment for upload to complete
    await new Promise(resolve => setTimeout(resolve, 1000));
    
    updateOTAStatus('installing');
    
    // Start OTA update with uploaded file
    const otaFormData = new FormData();
    otaFormData.append('filename', file.name);
    
    const otaResponse = await fetch('/ota/update', {
      method: 'POST',
      body: otaFormData
    });
    
    if (!otaResponse.ok) {
      throw new Error('Napaka pri posodobitvi: ' + await otaResponse.text());
    }
    
    const result = await otaResponse.text();
    console.log('OTA update started:', result);
    
    // Clear file input
    fileInput.value = '';
    
  } catch (error) {
    console.error('Upload and OTA error:', error);
    alert('Napaka: ' + error.message);
    updateOTAStatus('failed');
  }
}

// Start OTA from local file
async function startLocalOTA() {
  const select = document.getElementById('firmwareSelect');
  const filename = select.value;
  
  if (!filename) {
    alert('Prosim izberite firmware datoteko');
    return;
  }
  
  if (!confirm(`Ali ste prepričani, da želite posodobiti firmware iz datoteke:\n${filename}\n\nNaprava se bo ponovno zagnala.`)) {
    return;
  }
  
  try {
    const formData = new FormData();
    formData.append('filename', filename);
    
    const response = await fetch('/ota/update', {
      method: 'POST',
      body: formData
    });
    
    if (!response.ok) {
      throw new Error(await response.text());
    }
    
    const result = await response.text();
    console.log('OTA update started:', result);
    
  } catch (error) {
    console.error('OTA update error:', error);
    alert('Napaka pri posodobitvi: ' + error.message);
    updateOTAStatus('failed');
  }
}

// Load OTA status information
async function loadOTAStatus() {
  try {
    const response = await fetch('/ota/status');
    if (!response.ok) return;
    
    const data = await response.json();
    
    document.getElementById('currentPartition').textContent = data.current_partition || 'Neznano';
    document.getElementById('freeSketchSpace').textContent = formatBytes(data.free_sketch_space || 0);
    document.getElementById('sketchSize').textContent = formatBytes(data.sketch_size || 0);
    document.getElementById('fatfsSpace').textContent = formatBytes(data.free_fatfs || 0) + ' / ' + formatBytes(data.total_fatfs || 0);
    
  } catch (error) {
    console.error('Error loading OTA status:', error);
  }
}

// Load firmware files from FATFS
async function loadFirmwareFiles() {
  try {
    console.log('Loading firmware files from FATFS...');
    const response = await fetch('/api/list?storage=fatfs&path=/');
    console.log('Response status:', response.status);
    console.log('Response ok:', response.ok);
    
    if (!response.ok) {
      console.error('Failed to load file list:', response.status, response.statusText);
      const errorText = await response.text();
      console.error('Error response:', errorText);
      return;
    }
    
    const data = await response.json();
    console.log('FATFS file list:', data);
    console.log('Data type:', typeof data);
    console.log('Data length:', data.length);
    
    const select = document.getElementById('firmwareSelect');
    
    select.innerHTML = '<option value="">Izberi firmware datoteko...</option>';
    
    if (data && data.length > 0) {
      console.log('Found files:', data.length);
      data.forEach(file => {
        console.log('Checking file:', file.name, file.type);
        if (file.name && file.name.endsWith('.bin')) {
          const option = document.createElement('option');
          option.value = file.name;
          option.textContent = file.name + ' (' + formatBytes(file.size) + ')';
          select.appendChild(option);
          console.log('Added .bin file:', file.name);
        }
      });
    } else {
      console.log('No files found or empty response');
    }
    
  } catch (error) {
    console.error('Error loading firmware files:', error);
  }
}

// Format bytes to human readable format
function formatBytes(bytes) {
  if (bytes === 0) return '0 B';
  const k = 1024;
  const sizes = ['B', 'KB', 'MB', 'GB'];
  const i = Math.floor(Math.log(bytes) / Math.log(k));
  return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + ' ' + sizes[i];
}

loadFiles('/', 'fatfs');
loadStorageStats();

// Initialize OTA functionality
initOTAWebSocket();
loadOTAStatus();
loadFirmwareFiles();
</script>
</body>
</html>
   )rawliteral";

#endif // NADZORNIKPROSTORA_H
