function Original_Map = FuncUpdateOriginalMap(Original_Map,Pose,Scan,SMOOTHING_MODE,KERNEL_SIZE,MODE_DERIVATIVES,MODE_MAP)

Original_Map = FuncInitialiseGridMap(Original_Map,Pose,Scan);
Original_Map = FuncSmoothN2(Original_Map,10,SMOOTHING_MODE,KERNEL_SIZE);
Original_Map = FuncMapGrid(Original_Map,MODE_DERIVATIVES,MODE_MAP);

end