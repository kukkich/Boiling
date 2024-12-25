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
using SharpMath.Geometry._2D.Сylinder;
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
    // services.AddTransient<ISLAESolver<SparseMatrix>, LUSparseThroughProfileConversion>();

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

    const double r = 5;
    const double h = 6;

    var water = new RectArea(
        new Rectangle(
            1, 0,
            r, h
        ),
        materialId: 0
    );

    var areas = new AreasMaterialSetterFactory(
        [water],
        defaultMaterialIdId: 0
    );

    var nestingDegree = 1;
    
    var grid = new GridBuilder()
        .SetXAxis(new AxisSplitParameter(
            [1, r],
            new UniformSplitter(2)
        ))
        .SetYAxis(new AxisSplitParameter(
            [0, h], 
            new UniformSplitter(2)
        ))
        .SetMaterialSetterFactory(areas)
        .Build();

    //var velocityParameter = new ConvectionVelocity(grid.Nodes, 0.001);
    //for (var i = 0; i < grid.Nodes.TotalPoints; i++)
    //{
    //    var velocity = velocityParameter.Get(grid.Nodes[i]);
    //    Console.WriteLine($"{grid.Nodes[i].X:F5} {grid.Nodes[i].Y:F5} {velocity.X:E5} {velocity.Y:E5}");
    //}
    
    var materialProvider = new BoilingMaterialProvider([
        new BoilingMaterial(1, 1, 1d)
        //new BoilingMaterial(0.6, 999.97, 4200d)
    ]);

    var solver = provider.GetRequiredService<BoilingDirectSolver>();
    solver.Allocate(grid);
    solver.Allocate(materialProvider);
    solver.Allocate(new UniformSplitter(50 * nestingDegree)
        .EnumerateValues(new Interval(0d, 50))
        .ToArray());

    var tempValues = new double[grid.Nodes.TotalPoints];

    var u = new Func<Point, double, double>((p, t) => p.R() * p.R() + p.Z() + t);

    for (int i = 0; i < tempValues.Length; i++)
    {
        tempValues[i] = u(grid.Nodes[i], 0);
    }

    var femSolution = solver.Solve(new Vector(tempValues));

    var outputGrid = new GridBuilder()
        .SetXAxis(new AxisSplitParameter(
            [0, r],
            new UniformSplitter(85)
        ))
        .SetYAxis(new AxisSplitParameter(
            [0, h], 
            new UniformSplitter(140)
        ))
        .SetMaterialSetterFactory(areas)
        .Build();

    var values = new double[outputGrid.Nodes.TotalPoints];

    //for (var i = 0; i < outputGrid.Nodes.TotalPoints; i++)
    //{
    //    values[i] = femSolution.Calculate(outputGrid.Nodes[i], 100);
    //    Console.WriteLine($"{outputGrid.Nodes[i].X:F5} {outputGrid.Nodes[i].Y:F5} {values[i]:E5}");
    //}

    //var values = new double[grid.Nodes.TotalPoints];

    //for (var i = 0; i < grid.Nodes.TotalPoints; i++)
    //{
    //    values[i] = femSolution.Calculate(grid.Nodes[i], 1d);
    //    Console.WriteLine($"{grid.Nodes[i].X:F5} {grid.Nodes[i].Y:F5} {values[i]:E5}");
    //}

    var filePath = "output.txt";

    using (var writer = new StreamWriter(filePath))
    {
        for (var i = 0; i < outputGrid.Nodes.TotalPoints; i++)
        {
            values[i] = femSolution.Calculate(outputGrid.Nodes[i], 50d);
            writer.WriteLine($"{outputGrid.Nodes[i].X:F5} {outputGrid.Nodes[i].Y:F5} {values[i]:E5}");
        }
    }

    //Console.WriteLine(values.Any(v => v < 0));
}

RunBoiling();