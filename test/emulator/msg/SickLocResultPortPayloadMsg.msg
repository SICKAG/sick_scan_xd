# Definition of ros message SickLocResultPortPayloadMsg.
# SickLocResultPortPayloadMsg publishes the payload of a result port telegram
# for sick localization (52 byte). See chapter 5.9 ("About result port telegrams")
# of the operation manual for further details.

uint16 errorcode      # ErrorCode 0: OK, ErrorCode 1: UNKNOWNERROR. Size: UInt16 = 2 byte
uint32 scancounter    # Counter of related scan data. Size: UInt32 = 4 byte
uint32 timestamp      # Time stamp of the pose [ms]. The time stamp indicates the time at which the pose is calculated. Size: UInt32 = 4 byte
int32  posex          # Position X of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
int32  posey          # Position Y of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
int32  poseyaw        # Orientation (yaw) of the vehicle on the map [mdeg] Size: Int32 = 4 byte
uint32 reserved1      # Reserved. Size: UInt32 = 4 byte
int32  reserved2      # Reserved. Size: Int32 = 4 byte
uint8  quality        # Quality of pose [0 ... 100], 1 = bad pose quality, 100 = good pose quality. Size: UInt8 = 1 byte
uint8  outliersratio  # Ratio of beams that cannot be assigned to the current reference map [%]. Size: UInt8 = 1 byte
int32  covariancex    # Covariance c1 of the pose X [mm^2]. Size: Int32 = 4 byte
int32  covariancey    # Covariance c5 of the pose Y [mm^2]. Size: Int32 = 4 byte
int32  covarianceyaw  # Covariance c9 of the pose Yaw [mdeg^2]. Size: Int32 = 4 byte
uint64 reserved3      # Reserved. Size: UInt64 = 8 byte

