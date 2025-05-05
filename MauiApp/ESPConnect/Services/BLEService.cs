using System.Diagnostics;
using Plugin.BLE;
using Plugin.BLE.Abstractions;
using Plugin.BLE.Abstractions.Contracts;
using Plugin.BLE.Abstractions.EventArgs;
using Plugin.BLE.Abstractions.Exceptions;
using Plugin.BLE.Abstractions.Extensions;

namespace ESPConnect.Services
{
    public partial class BLEService : ObservableObject
    {
        bool IsLedOn = false;
        // The Bluetooth manager (handles overall state)
        private readonly IBluetoothLE _bluetoothManager;

        // The adapter (handles scanning, connecting, etc.)
        protected IAdapter Adapter;
        public IDevice _esp32Device;
        public Guid ESPguid;
        private Guid _serviceUuid = new Guid("0000ABF2-0000-1000-8000-00805f9b34fb");
        private Guid _characteristicUuid = new Guid("0000ABF3-0000-1000-8000-00805f9b34fb");
        public Action OnDeviceDisconnectedAction { get; set; }

        [ObservableProperty]
        public bool isScanning = false;
        public bool IsStateOn => _bluetoothManager.IsOn;

        CancellationTokenSource _scanCancellationTokenSource = null;

        public BLEService()
        {
            // Initialize the Bluetooth manager and adapter
            Debug.WriteLine("Inside BLE Service constructor");
            _bluetoothManager = CrossBluetoothLE.Current;
            Adapter = _bluetoothManager?.Adapter;

            if (_bluetoothManager is null)
            {
                Debug.WriteLine("BluetoothManager is null");
                
            }
            else if (Adapter is null)
            {
                Debug.WriteLine("Adapter is null");
            }
            else
            {
                ConfigureBLE();
            } 
        }

        public async Task<bool> BLESend(double lat, double lon, long timestamp)
        {
            Debug.WriteLine($"Test From BL send");
            double latitude = lat;
            double longitude = lon;
            double currentUnixTime = timestamp;

            Debug.WriteLine($"Sending- lat:{latitude}, lon:{longitude}, unixTime:{currentUnixTime}");

            try
            {
                // 1. Get the BLE service from the connected device
                var service = await _esp32Device.GetServiceAsync(_serviceUuid);
                if (service == null)
                {
                    Debug.WriteLine("Service not found on the ESP32 device.");
                    await Shell.Current.DisplayAlert("", "Need connection to device First.", "OK");
                    return false;
                }

                // 2. Get the characteristic we can write to
                var characteristic = await service.GetCharacteristicAsync(_characteristicUuid);
                if (characteristic == null)
                {
                    Debug.WriteLine("Writable characteristic not found on the ESP32 device.");
                    await Shell.Current.DisplayAlert("", "Need connection to device First.", "OK");
                    return false;
                }

                // 3. Format the data as a simple text string (e.g., CSV-like)
                string dataString = $"{latitude},{longitude},{currentUnixTime}";
                byte[] dataBytes = System.Text.Encoding.UTF8.GetBytes(dataString);

                // 4. Write the bytes to the characteristic
                await characteristic.WriteAsync(dataBytes);

                Debug.WriteLine($"Sent data to ESP: {dataString}");
                return true;
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Error sending coordinates: {ex.Message}");
                await Shell.Current.DisplayAlert("", "Error, could not send data", "OK");
                return false;
            }
        }

        public async Task<bool> BLEConnect()
        {
            Debug.WriteLine($"Test From BLE connect");
            if (_esp32Device != null && _esp32Device.State == DeviceState.Connected)
            {
                // Already connected! 
                Debug.WriteLine("ESP32 is already connected. No need to scan again.");
                return true;
            }

            if (!IsStateOn)
            {
                await Shell.Current.DisplayAlert("", "Bluetooth is not ON.\nPlease turn on Bluetooth and try again.", "OK");
                IsScanning = false;
                return false;
            }
            if (!await HasCorrectPermissions())
            {
                Debug.WriteLine("Aborting scan attempt");
                IsScanning = false;
                return false;
            }
            Debug.WriteLine(GetStateText());

            Debug.WriteLine("Starting scan");
            //await UpdateConnectedDevices();

            _scanCancellationTokenSource = new();
            Debug.WriteLine("call Adapter.StartScanningForDevicesAsync");
            await Adapter.StartScanningForDevicesAsync(_scanCancellationTokenSource.Token);
            Debug.WriteLine("back from Adapter.StartScanningForDevicesAsync");
            _scanCancellationTokenSource.Dispose();
            _scanCancellationTokenSource = null;
            IsScanning = false;

            
            Debug.WriteLine($"Guid: {ESPguid}");

            if (_esp32Device == null)
            {
                Debug.WriteLine("No ESP32 device has been discovered yet.");
                return false;
            }
            Debug.WriteLine($"Attempting connect");

            // Create a cancellation token if needed
            using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(10));

            try
            {
                await Adapter.ConnectToDeviceAsync(_esp32Device, new ConnectParameters(forceBleTransport: true), cts.Token);
                Debug.WriteLine("Connection established to ESP32!");
            }
            catch (DeviceConnectionException ex)
            {
                Debug.WriteLine($"Could not connect to device, error: {ex}");
                return false;
            }
            catch (OperationCanceledException)
            {
                Debug.WriteLine("Connection attempt was canceled or timed out.");
                return false;
            }

            return true;
        }

        public void ConfigureBLE()
        {
            Debug.WriteLine("Configuring BLE...");

            // Subscribe to the Bluetooth state changes
            _bluetoothManager.StateChanged += OnBluetoothStateChanged;

            // Set up scanner
            Adapter.ScanMode = ScanMode.LowLatency;
            Adapter.ScanTimeout = 30000; // ms

            // Subscribe to adapter events
            Adapter.ScanTimeoutElapsed += Adapter_ScanTimeoutElapsed;
            Adapter.DeviceAdvertised += OnDeviceAdvertised;
            Adapter.DeviceDiscovered += OnDeviceDiscovered;
            Adapter.DeviceConnectionLost += OnDeviceDisconnected;

            Debug.WriteLine("Configuring BLE... DONE");
        }

        // -------------------- Event Handlers --------------------
        private void OnDeviceDisconnected(object sender, EventArgs e)
        {
            Debug.WriteLine("Device disconnected");
            OnDeviceDisconnectedAction?.Invoke();
            // Cleanup will happen inside ScanForDevicesAsync
        }
        private void OnBluetoothStateChanged(object sender, BluetoothStateChangedArgs e)
        {
            Debug.WriteLine("State changed");
            Debug.WriteLine("OnBluetoothStateChanged from " + e.OldState + " to " + e.NewState);
        }

        private void Adapter_ScanTimeoutElapsed(object sender, EventArgs e)
        {
            Debug.WriteLine("Adapter_ScanTimeoutElapsed");
            // Cleanup will happen inside ScanForDevicesAsync
        }

        private void OnDeviceAdvertised(object sender, DeviceEventArgs e)
        {
            Debug.WriteLine($"Device advertised: {e.Device.Name} (ID: {e.Device.Id})");
            // Handle device advertisement
        }

        private void OnDeviceDiscovered(object sender, DeviceEventArgs e)
        {
            Debug.WriteLine($"Found device: {e.Device.Name} with ID: {e.Device.Id}");
            if (e.Device != null && e.Device.Name == "ESP32_BLE_Device")
            {
                Debug.WriteLine($"Found device: {e.Device.Name} with ID: {e.Device.Id}");
                
                // Save the device or its ID for later use
                _esp32Device = e.Device;
                ESPguid = e.Device.Id;

                Debug.WriteLine("Found ESP device, attempting to shutdown scan");
                _scanCancellationTokenSource?.Cancel();
            }
        }

        private string GetStateText()
        {
            var result = "Unknown BLE state.";
            switch (_bluetoothManager.State)
            {
                case BluetoothState.Unknown:
                    result = "Unknown BLE state.";
                    break;
                case BluetoothState.Unavailable:
                    result = "BLE is not available on this device.";
                    break;
                case BluetoothState.Unauthorized:
                    result = "You are not allowed to use BLE.";
                    break;
                case BluetoothState.TurningOn:
                    result = "BLE is warming up, please wait.";
                    break;
                case BluetoothState.On:
                    result = "BLE is on.";
                    break;
                case BluetoothState.TurningOff:
                    result = "BLE is turning off. That's sad!";
                    break;
                case BluetoothState.Off:
                    result = "BLE is off. Turn it on!";
                    break;
            }
            return result;
        }

        private async Task<bool> HasCorrectPermissions()
        {
            Debug.WriteLine("Verifying Bluetooth permissions..");
            var permissionResult = await Permissions.CheckStatusAsync<Permissions.Bluetooth>();
            if (permissionResult != PermissionStatus.Granted)
            {
                permissionResult = await Permissions.RequestAsync<Permissions.Bluetooth>();
            }
            Debug.WriteLine($"Result of requesting Bluetooth permissions: '{permissionResult}'");
            if (permissionResult != PermissionStatus.Granted)
            {
                Debug.WriteLine("Permissions not available, direct user to settings screen.");
                Debug.WriteLine("Permission denied. Not scanning.");
                AppInfo.ShowSettingsUI();
                return false;
            }

            return true;
        }
        private async Task UpdateConnectedDevices()
        {
            foreach (var connectedDevice in Adapter.ConnectedDevices)
            {
                //update rssi for already connected devices (so that 0 is not shown in the list)
                try
                {
                    await connectedDevice.UpdateRssiAsync();
                }
                catch (Exception ex)
                {
                    Debug.WriteLine($"Failed to update RSSI for {connectedDevice.Name}. Error: {ex.Message}");
                }

                //AddOrUpdateDevice(connectedDevice);
            }
        }


    }
}
