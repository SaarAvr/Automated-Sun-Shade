namespace ESPConnect.View;

public partial class MainPage : ContentPage
{
    int count = 0;
    private BLEService _bleService;

    public MainPage(BaseViewModel viewModel)
    {
        InitializeComponent();
        BindingContext = viewModel;

        // Instantiate the BLEService
        //_bleService = new BLEService();
    }
}
