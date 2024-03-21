%% First-time script for building a 12s2p pouch geometry battery module. Not taken into account the specific cell specs

import simscape.battery.builder.*

pouchGeometry = PouchGeometry(Length=simscape.Value(0.25,"m")); % TODO: Find data on LG chem NCM 712 Pouch
batteryCell = Cell(Geometry=PouchGeometry)

batteryCell.CellModelOptions.BlockParameters.thermal_port = "model";

batteryparallelassembly = ParallelAssembly(Cell=batteryCell,...
    NumParallelCells=2, ...
    ModelResolution="Detailed")


batteryModule = Module(ParallelAssembly=batteryparallelassembly,...
    NumSeriesAssemblies=12, ...
    InterParallelAssemblyGap=simscape.Value(0.005,"m"), ...
    ModelResolution="Detailed", ...
    AmbientThermalPath="CellBasedThermalResistance")

disp(batteryModule.NumModels);
f = uifigure(Color="w");
tl = tiledlayout(1,2,"Parent",f,"TileSpacing","Compact");
nexttile(tl)
batteryModuleChart1 = BatteryChart(Parent=tl,Battery=batteryModule);
nexttile(tl)
batteryModuleChart2 = BatteryChart(Parent=tl,Battery=batteryModule,SimulationStrategyVisible="On");

buildBattery(batteryModule,LibraryName="pouchModuleLibrary");

batteryBuilder