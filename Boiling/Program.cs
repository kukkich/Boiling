﻿using Boiling.DirectSolver;
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

    const double r = 0.08;
    const double h = 0.09;

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
            new UniformSplitter(80 * nestingDegree)
        ))
        .SetYAxis(new AxisSplitParameter(
            [0, h], 
            new UniformSplitter(80 * nestingDegree)
        ))
        .SetMaterialSetterFactory(areas)
        .Build();

    var materialProvider = new BoilingMaterialProvider([
        new BoilingMaterial(0.6, 999.97, 4200d)
    ]);

    var solver = provider.GetRequiredService<BoilingDirectSolver>();
    solver.Allocate(grid);
    solver.Allocate(materialProvider);
    solver.Allocate(new UniformSplitter(200 * nestingDegree)
        .EnumerateValues(new Interval(0d, 360d))
        .ToArray());

    var femSolution = solver.Solve(Vector.Create(grid.Nodes.TotalPoints, 25));

    for (var i = 0; i < grid.Nodes.TotalPoints; i++)
    {
        var u = femSolution.Calculate(grid.Nodes[i], 2);
        Console.WriteLine($"{grid.Nodes[i].X:F5} {grid.Nodes[i].Y:F5} {u:E5}");
    }
}

RunBoiling();