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

    private void OnCounterClicked(object sender, EventArgs e)
    {
        count++;

        if (count == 1)
            CounterBtn.Text = $"Clicked {count} time";
        else
            CounterBtn.Text = $"Clicked {count} times";

        SemanticScreenReader.Announce(CounterBtn.Text);
    }
}
