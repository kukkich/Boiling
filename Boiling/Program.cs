using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging;
using Serilog;
using SharpMath.FiniteElement.Materials.HarmonicWithoutChi;
using SharpMath.FiniteElement.Materials.MaterialSetter.Areas;
using SharpMath.FiniteElement.Materials.Providers;
using SharpMath.Geometry._2D;
using SharpMath.Geometry.Splitting;

void ConfigureServices(IServiceCollection services)
{
    IConfiguration configuration = new ConfigurationBuilder()
        .SetBasePath(Directory.GetCurrentDirectory())
        .AddJsonFile("appsettings.json", optional: false, reloadOnChange: true)
        .Build();
    services.AddSingleton(configuration);

    Log.Logger = new LoggerConfiguration()
        .ReadFrom.Configuration(configuration)
        .Enrich.FromLogContext()
        .CreateLogger();
    services.AddLogging(loggingBuilder =>
        loggingBuilder.AddSerilog(dispose: true));
}

void RunBoiling()
{
    var services = new ServiceCollection();
    ConfigureServices(services);
    var provider = services.BuildServiceProvider();

    var logger = provider.GetRequiredService<ILogger<Program>>();
    logger.LogInformation("Boiling, You're just a miserable copy of me!");
    logger.LogCritical("No, I'm the upgrade!");

    const double r = 0.08;
    const double h = 0.09;

    var water = new RectArea(
        new Rectangle(
            0, 0,
            r, h
        ),
        materialId: 1
    );

    var areas = new AreasMaterialSetterFactory(
        [water],
        defaultMaterialIdId: 0
    );

    const int nestingDegree = 1;
    
    var grid = new GridBuilder()
        .SetXAxis(new AxisSplitParameter(
            [0, r],
            new UniformSplitter(5 * nestingDegree)
        ))
        .SetYAxis(new AxisSplitParameter(
            [0, h], 
            new UniformSplitter(10 * nestingDegree)
        ))
        .SetMaterialSetterFactory(areas)
        .Build();
    var materialProvider = new FromArrayMaterialProvider([
        new Material(0, 0),
        new Material(1, 1e-3),
    ]);
}