using Boiling.DirectSolver;
using Boiling.FiniteElement.Time;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging;
using Serilog;
using SharpMath.EquationsSystem.Preconditions;
using SharpMath.EquationsSystem.Solver;
using SharpMath.FiniteElement;
using SharpMath.FiniteElement._2D.Parameters;
using SharpMath.FiniteElement.Materials.Boiling;
using SharpMath.FiniteElement.Materials.HarmonicWithoutChi;
using SharpMath.FiniteElement.Materials.MaterialSetter.Areas;
using SharpMath.FiniteElement.Materials.Providers;
using SharpMath.Geometry;
using SharpMath.Geometry._2D;
using SharpMath.Geometry.Splitting;
using SharpMath.Matrices.Sparse;
using SharpMath.Vectors;

void ConfigureServices(IServiceCollection services)
{
    IConfiguration configuration = new ConfigurationBuilder()
        .SetBasePath(Directory.GetCurrentDirectory())
        .AddJsonFile("appsettings.json", optional: false, reloadOnChange: true)
        .Build();
    services.AddSingleton(configuration);

    services.AddScoped<LocalOptimalSchemeConfig>(provider =>
    {
        provider.GetService<IConfiguration>();
        var losConfig = configuration
            .GetSection("Boiling")
            .GetSection("LOS")
            .Get<LocalOptimalSchemeConfig>();

        return losConfig!;
    });

    services.AddTransient<BoilingDirectSolver>();
    services.AddTransient<LUSparseThroughProfileConversion>();
    
    services.AddTransient<ISLAESolver<SparseMatrix>, LocalOptimalScheme>();
    services.AddTransient<LUPreconditioner>();
    services.AddTransient<SparsePartialLUResolver>();

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

    const double r = 2;
    const double h = 2;

    var water = new RectArea(
        new Rectangle(
            0, 0,
            r, h
        ),
        materialId: 0
    );

    var areas = new AreasMaterialSetterFactory(
        [water],
        defaultMaterialIdId: 0
    );

    const int nestingDegree = 1;
    
    var grid = new GridBuilder()
        .SetXAxis(new AxisSplitParameter(
            [0, r],
            new UniformSplitter(1 * nestingDegree)
        ))
        .SetYAxis(new AxisSplitParameter(
            [0, h], 
            new UniformSplitter(1 * nestingDegree)
        ))
        .SetMaterialSetterFactory(areas)
        .Build();

    var materialProvider = new BoilingMaterialProvider([
        new BoilingMaterial(1d, 1d, 1d)
    ]);

    var solver = provider.GetRequiredService<BoilingDirectSolver>();
    solver.Allocate(grid);
    solver.Allocate(materialProvider);
    solver.Allocate(new UniformSplitter(2 * nestingDegree)
        .EnumerateValues(new Interval(0d, 2d))
        .ToArray());

    var femSolution = solver.Solve(new Vector(new double[grid.Nodes.TotalPoints]));
}

RunBoiling();