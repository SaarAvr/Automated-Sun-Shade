using System.Timers;

namespace ESPConnect.ViewModel;

public partial class BaseViewModel : ObservableObject
{
    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(IsNotBusy))]
    bool isBusy;

    [ObservableProperty]
    string title;

    [ObservableProperty]
    public bool locationStatus;

    [ObservableProperty]
    public static bool connectStatus;

    [ObservableProperty]
    public bool sendStatus;

    private System.Timers.Timer _timer;

    [ObservableProperty]
    public long currentUnixTime;

    [ObservableProperty]
    public double latitude;

    [ObservableProperty]
    public double longitude;

    public bool IsNotBusy => !IsBusy;

    IGeolocation geolocation;
    private BLEService _bleService = null;

    public string SomeProperty { get; set; } = "Hello MVVM!";

    public BaseViewModel(IGeolocation geolocation, BLEService bleService)
    {
        Console.WriteLine("----------BaseViewModel Constructor----------");
        Title = "GPS app";
        LocationStatus = false;
        this.geolocation = geolocation;

        // Initialize the timer and set it to update every second (1000ms)
        _timer = new System.Timers.Timer(1000);
        _timer.Elapsed += OnTimerElapsed;
        _timer.Start();

        // Set the initial time
        currentUnixTime = GetCurrentUnixTime();

        _bleService = bleService;
        _bleService.OnDeviceDisconnectedAction = () =>
        {
            ConnectStatus = false;
            Debug.WriteLine("Disconnected! ConnectStatus set to false.");
        };
    }

    [RelayCommand]
    public async Task Connect()
    {
        Debug.WriteLine("------Inside Connect BaseViewModel");
        ConnectStatus = await _bleService.BLEConnect();
        Debug.WriteLine("------Inside Connect BaseViewModel, after BLE");
    }

    [RelayCommand]
    public async Task Send()
    {
        Debug.WriteLine("------Inside Send BaseViewModel");
        SendStatus = await _bleService.BLESend(Latitude, Longitude, CurrentUnixTime);
        Debug.WriteLine("------Inside Send BaseViewModel, after BLE");
    }

    [RelayCommand]
    public async Task GetCurrentLocation()
    {
        Debug.WriteLine("------Inside function");

        try
        {
            var status = await Permissions.CheckStatusAsync<Permissions.LocationWhenInUse>();
            if (status != PermissionStatus.Granted)
            {
                status = await Permissions.RequestAsync<Permissions.LocationWhenInUse>();
            }

            // Get cached location, else get real location.
            var location = await geolocation.GetLastKnownLocationAsync();
            if (location == null)
            {
                location = await geolocation.GetLocationAsync(new GeolocationRequest
                {
                    DesiredAccuracy = GeolocationAccuracy.Medium,
                    Timeout = TimeSpan.FromSeconds(30)
                });
            }
            if (location != null)
            {
                Latitude = location.Latitude;
                Longitude = location.Longitude;
                LocationStatus = true;

                await Shell.Current.DisplayAlert("", $"Latitude: {location.Latitude}" +
                    $"\nLongitude: {location.Longitude}" +
                    $"\nUnix Time: {CurrentUnixTime}", "OK");
            } else
            {
                LocationStatus = false;
                Console.WriteLine("Location is null. GPS might be disabled.");
            }



            
            
                
        }
        catch (FeatureNotEnabledException)
        {
            await Shell.Current.DisplayAlert("Error", "Please enable location services.", "OK");
        }
        catch (Exception ex)
        {
            Debug.WriteLine($"Unable to query location: {ex.Message}");
            await Shell.Current.DisplayAlert("Error!", ex.Message, "OK");
        }
    }

    public long GetCurrentUnixTime()
    {
        var unixTime = DateTimeOffset.UtcNow.ToUnixTimeSeconds();
        return unixTime;
    }

    private void OnTimerElapsed(object sender, ElapsedEventArgs e)
    {
        // Update the time every second
        CurrentUnixTime = GetCurrentUnixTime();
    }

}
