# Changelog

## [0.5.0](https://github.com/neurobionics/onshape-robotics-toolkit/compare/onshape-robotics-toolkit-v0.4.0...onshape-robotics-toolkit-v0.5.0) (2026-01-16)


### Features

* add support for composite parts ([1b317c1](https://github.com/neurobionics/onshape-robotics-toolkit/commit/1b317c184227d3a85fb58abd89ea3f75fc53df72))


### Bug Fixes

* add MJCF robot methods to config and update examples ([19329ad](https://github.com/neurobionics/onshape-robotics-toolkit/commit/19329ad787943cfcbfc4b9bacb334941c5cb477c))
* revert set variable method and validate variables argument before building payload ([25f0883](https://github.com/neurobionics/onshape-robotics-toolkit/commit/25f08839e92e57c3cbbe2a190d244c32a9087dd9))
* update variables config to accomodate the set_variables fix ([c5d35f4](https://github.com/neurobionics/onshape-robotics-toolkit/commit/c5d35f492b89aeb97bbf17e32b69de555a566863))

## [0.4.0](https://github.com/neurobionics/onshape-robotics-toolkit/compare/onshape-robotics-toolkit-v0.3.0...onshape-robotics-toolkit-v0.4.0) (2025-10-15)


### Features

* create formats for saving MJCF/URDF and handling conversions ([731f645](https://github.com/neurobionics/onshape-robotics-toolkit/commit/731f64505913fe83fe8455883e0715bff1d70e3d))
* create formats for saving MJCF/URDF and handling conversions ([047159b](https://github.com/neurobionics/onshape-robotics-toolkit/commit/047159b4bccfc2f57afd9d3b3479515d5cf7c2af))

## [0.3.0](https://github.com/neurobionics/onshape-robotics-toolkit/compare/onshape-robotics-toolkit-v0.2.1...onshape-robotics-toolkit-v0.3.0) (2025-10-15)


### Features

* add a robust key naming system and a CAD class to act as a unified holding entity for instances, occurrences, and ids ([e93be91](https://github.com/neurobionics/onshape-robotics-toolkit/commit/e93be91b4613be742cfcfcea9df8b197bd222789))
* add API call counter to Client class and estimate_api_calls method ([dbd9bf9](https://github.com/neurobionics/onshape-robotics-toolkit/commit/dbd9bf9e02ceff7d77b57855e302256070e62621))
* add API call counter to Client class and estimate_api_calls method to CAD class ([a5682f0](https://github.com/neurobionics/onshape-robotics-toolkit/commit/a5682f0139973c0957d97f66021f64c3c1a821ae))
* add codecov and fix URDF/urdf case mismatch ([e54a38a](https://github.com/neurobionics/onshape-robotics-toolkit/commit/e54a38a4ebe07b109f8e25c2e0cff26559a951e1))
* add config setup and a global auto-save feature ([feb521b](https://github.com/neurobionics/onshape-robotics-toolkit/commit/feb521b26b1ada97873ed56a90a2c7d52be91ecd))
* add custom mesh and file path support for robot.save ([38f2a53](https://github.com/neurobionics/onshape-robotics-toolkit/commit/38f2a534f4825d48039acdaf1078eab46953b561))
* add custom mesh and file path support for robot.save ([943fe14](https://github.com/neurobionics/onshape-robotics-toolkit/commit/943fe14f2b50a1edb75dc227599676505cdab4c2))
* add floating point cleanup ([bd4459d](https://github.com/neurobionics/onshape-robotics-toolkit/commit/bd4459dc4886b418b851d365bc173ece2a12f7cf))
* add from_graph method to robot class ([73fd747](https://github.com/neurobionics/onshape-robotics-toolkit/commit/73fd747325e4d14debbe80bef440d124b98572db))
* add helper methods to conviniently configure loguru ([ca0c684](https://github.com/neurobionics/onshape-robotics-toolkit/commit/ca0c68444310a355b16b2373cdd1a79c2af19aef))
* add initial attempts at parsing mate patterns; transformations dont work ([8184a7d](https://github.com/neurobionics/onshape-robotics-toolkit/commit/8184a7dcc4d6bdbc3d11d0e550a5433a26caf958))
* add kinematic tree class tohelp with graph processing ([320d978](https://github.com/neurobionics/onshape-robotics-toolkit/commit/320d9782b181585bd61a942188e03e26855575bf))
* add mate processing logic w/o tests ([683688d](https://github.com/neurobionics/onshape-robotics-toolkit/commit/683688d53056a1873baf8f28660d9e61bbb7c85c))
* add method to ensure mate names are unique before URDF output ([a8394ae](https://github.com/neurobionics/onshape-robotics-toolkit/commit/a8394ae0606fe097ae5a8486e39661a12c50a99d))
* add new tuple-based keys instead of strings ([9924ddf](https://github.com/neurobionics/onshape-robotics-toolkit/commit/9924ddfbcc5ac40d233427945acbf3f114e8ba06))
* add plotting methods and fetch flexible subassemblies ([31a62ba](https://github.com/neurobionics/onshape-robotics-toolkit/commit/31a62badcae7d476cf091fcae41b9bd55f787a01))
* add remap_mates method to graph class ([0305bd2](https://github.com/neurobionics/onshape-robotics-toolkit/commit/0305bd2f036f5e4262af402668a3ce840529027f))
* add rigid-subassembly logic to mate parsing ([43acbd4](https://github.com/neurobionics/onshape-robotics-toolkit/commit/43acbd43a3d3a0ba3b0d3cc115308f77372029f4))
* add rigidAssemblyToPartTF to parts ([5744d07](https://github.com/neurobionics/onshape-robotics-toolkit/commit/5744d07ab70c55c4b3a3e8697f40e474a1ec13f1))
* add setup-uv-env action ([821fdee](https://github.com/neurobionics/onshape-robotics-toolkit/commit/821fdee2c96fda25ae7a1dc2e15c817ec0662cf5))
* add support and tests for mate groups ([96303d9](https://github.com/neurobionics/onshape-robotics-toolkit/commit/96303d9e9eb88042aeb862cd5cf45f9b3cee2916))
* add support and tests for mate groups ([ee63c04](https://github.com/neurobionics/onshape-robotics-toolkit/commit/ee63c04979db7603539a6716da2ca246384d3853))
* add support for assembly patterns that work both under root assembly and subassembly context ([c4d147f](https://github.com/neurobionics/onshape-robotics-toolkit/commit/c4d147f8699b48035bd8c048a086e1cdac292e7e))
* add tests and new CAD class with registries ([84e0325](https://github.com/neurobionics/onshape-robotics-toolkit/commit/84e03254d3fdcabe9cc00bffeb39f912b334aa68))
* add unit tests for parse, graph, and robot modules ([c4b5ee2](https://github.com/neurobionics/onshape-robotics-toolkit/commit/c4b5ee2fc33ba61dc73b487f37d53469dc61ff90))
* add uv files ([9684766](https://github.com/neurobionics/onshape-robotics-toolkit/commit/9684766e9cf5ef74260a5bcd10a5e446ea2c183f))
* all linting errors ([d282f7e](https://github.com/neurobionics/onshape-robotics-toolkit/commit/d282f7ea0cf60a1c41d1415d47a83f9882880f76))
* better transforms ([51e324d](https://github.com/neurobionics/onshape-robotics-toolkit/commit/51e324d780d5a5276b0c8c304056a82f47a242a4))
* bump max supported python version and fix TKinter windows error ([04a180f](https://github.com/neurobionics/onshape-robotics-toolkit/commit/04a180facd22062f871c5725486bd73723fd1373))
* flattened data structures for CAD ([d88d1e6](https://github.com/neurobionics/onshape-robotics-toolkit/commit/d88d1e6a349958b14ca9837ebcd6092e47ebcac8))
* initial patterns for CAD's unified key system ([f071fe8](https://github.com/neurobionics/onshape-robotics-toolkit/commit/f071fe8ab457a7b6bda0806dc2932005bd5cb1a0))
* integrate release-please ([02c1c11](https://github.com/neurobionics/onshape-robotics-toolkit/commit/02c1c11ee80923ade333ba30de258e69a2a350ee))
* make joint names unique ([bafbba6](https://github.com/neurobionics/onshape-robotics-toolkit/commit/bafbba615775227e6d42ac043df56170430b3edd))
* make KinematicGraph inherit DiGraph ([b6691a0](https://github.com/neurobionics/onshape-robotics-toolkit/commit/b6691a0ee922b4888688cd9b6d05e6063ce142de))
* populate parts and add part instances for every rigid subassembly ([95969ca](https://github.com/neurobionics/onshape-robotics-toolkit/commit/95969ca8848e1a8b2df9a2a17730263c413006dc))
* rename KinematicTree to KinematicGraph ([c60d454](https://github.com/neurobionics/onshape-robotics-toolkit/commit/c60d4545bdf1eb8b2659fc3ba67021d565601d21))
* replace old type-system and CAD class with new tested one ([9c54489](https://github.com/neurobionics/onshape-robotics-toolkit/commit/9c54489d8575acdb72afaba35e053913d2ce53af))
* restructure how part mass is fetched: switch to a two phase approach to not waste API calls ([8aef399](https://github.com/neurobionics/onshape-robotics-toolkit/commit/8aef399c89d3622a5faf5b1ee38f4f65e5ee21cc))
* reviewed parse module; removed all remapping methods ([59cc19f](https://github.com/neurobionics/onshape-robotics-toolkit/commit/59cc19f3ff629e6f60289e83fba2a9bac64d418b))
* update dev docs ([8a1f012](https://github.com/neurobionics/onshape-robotics-toolkit/commit/8a1f012281463dc20dfef917c6ec4f16d0e7a6d2))
* update KinematicGraph to be compatible with CAD ([7ac8075](https://github.com/neurobionics/onshape-robotics-toolkit/commit/7ac80753557d023dad909e89d44fbeeaea2f5c8e))
* update mate limits to config ([a310ff1](https://github.com/neurobionics/onshape-robotics-toolkit/commit/a310ff126b83ac6b5a443bbc3812d217836aa01b))
* update mate limits to config ([7764d1f](https://github.com/neurobionics/onshape-robotics-toolkit/commit/7764d1f84ee8447d02ad79cbb42ce31d84ca3e94))
* update Robot class to use PathKeys and inhert DiGraph ([67d9549](https://github.com/neurobionics/onshape-robotics-toolkit/commit/67d9549d95c343069af611f6f89d8f710f685fb2))
* use loguru instead of custom Logger class ([1a7ced8](https://github.com/neurobionics/onshape-robotics-toolkit/commit/1a7ced88d6503a2232418070cd14ae38ff7b1a01))
* uv init ([8a734b1](https://github.com/neurobionics/onshape-robotics-toolkit/commit/8a734b1c73368da85d38b3fa941abd9c1ac5f957))


### Bug Fixes

* add a search method to find the occurrence key to support patterns within sub-assemblies ([2a4ba81](https://github.com/neurobionics/onshape-robotics-toolkit/commit/2a4ba817d9ff7cd0990219ad6d531e7f6d1eb4a2))
* add expected URDF file for unit testing and modify gitignore ([60f2e9d](https://github.com/neurobionics/onshape-robotics-toolkit/commit/60f2e9d22d99891baaa969732378cb4fc04ed378))
* all mypy errors with parse module ([5c84220](https://github.com/neurobionics/onshape-robotics-toolkit/commit/5c842202580a3a63088b8d2e33b0007369e5b6ee))
* docs test ([37465f2](https://github.com/neurobionics/onshape-robotics-toolkit/commit/37465f2ecc8354600e97355ea3e748c25ab0b7cf))
* generate SUB_ASSEMBLY and MATE joiners to avoid conflict with part names ([3a37964](https://github.com/neurobionics/onshape-robotics-toolkit/commit/3a379641079b522710b1546f4032aa5cc66080e7))
* mate data order mismatch and extend remapping logic to matedOccurrence ([ad9a3a0](https://github.com/neurobionics/onshape-robotics-toolkit/commit/ad9a3a05d6e19c82d64316968964bd661f0f610c))
* minor bug that fails to set rigidAssemblyToPartTF ([915d2e6](https://github.com/neurobionics/onshape-robotics-toolkit/commit/915d2e6b6bf5b3bfdc7db87027814b39fc411fbe))
* models are free of linting errors ([9b0eee6](https://github.com/neurobionics/onshape-robotics-toolkit/commit/9b0eee6847759512c254d43ff99fdff2a574c6f3))
* more makefile errors ([9787c2b](https://github.com/neurobionics/onshape-robotics-toolkit/commit/9787c2b610b4c84a65ded235803730b1585f001f))
* more mypy static type checking fixes ([f4cfde4](https://github.com/neurobionics/onshape-robotics-toolkit/commit/f4cfde4907a8a5a55c71b83f03df2c9880c71381))
* mypy errors ([a95454f](https://github.com/neurobionics/onshape-robotics-toolkit/commit/a95454f265af5cd90d146e0e2a678a9d36296d97))
* occurrences and tf now works but transformations have issues ([84fc67a](https://github.com/neurobionics/onshape-robotics-toolkit/commit/84fc67a484567714d9b2a5af04b90f16c937f517))
* ocurrences now parse all available occurrence transforms from assembly json; mates at the root level that reference a part buried within a rigid-subassembly now have the correct transform ([206c2e2](https://github.com/neurobionics/onshape-robotics-toolkit/commit/206c2e2ca667f6c9def0bc50fb657993c334bd3f))
* Onshape's occurrence TF for subassemblies donot follow the same convention as parts, hence they cannot be used to compute part pose wrt rigid roots ([d0732a5](https://github.com/neurobionics/onshape-robotics-toolkit/commit/d0732a545c9458592de1605d55cb03f46e35ba19))
* part names with - throw errors in USD export ([22e5c4e](https://github.com/neurobionics/onshape-robotics-toolkit/commit/22e5c4e81b7bf452cee0677c438b58a29fc7f5cb))
* pattern joint locations are now calculated based on the other entity's tf; patterns now work ([6594144](https://github.com/neurobionics/onshape-robotics-toolkit/commit/6594144cd6f7d0bd6b97c7fb04f3a0d357782c3b))
* reference sub-assmebly child or parent correctly in mate pattern processing ([2a0c694](https://github.com/neurobionics/onshape-robotics-toolkit/commit/2a0c694ca3a8361d1f8cabc41f16c110ff82961b))
* remove old docs for log module ([f9d816b](https://github.com/neurobionics/onshape-robotics-toolkit/commit/f9d816b8ea0777e4f6f19e350a10f34921bb71cc))
* remove root part origin to COM initial transform; fix edge ordering in graph processing to match PARENT, CHILD convention ([b475b0b](https://github.com/neurobionics/onshape-robotics-toolkit/commit/b475b0bd33f322ecd2b335efb7ec2eb2116dd6fd))
* remove unused type ignores ([7d10c5f](https://github.com/neurobionics/onshape-robotics-toolkit/commit/7d10c5f38e10f976605b8df8ebe2eb82d47a305b))
* root level mates that refer parts within multiple levels of flex and rigid subassemblies now have correct transform ([6f2057a](https://github.com/neurobionics/onshape-robotics-toolkit/commit/6f2057a74be59f2e27ca82d0ee2099c859188a1d))
* separate building and processing graphs ([2e5d02b](https://github.com/neurobionics/onshape-robotics-toolkit/commit/2e5d02b8dfeed0a02da1cf7c39fb57f5b44bc749))
* subassemblies and parts now preserve the same structure as instances: we store instance duplicates for each occurrence ([260cfdc](https://github.com/neurobionics/onshape-robotics-toolkit/commit/260cfdc04bd74cb5f277d2b42c0e3a19b24c7aff))
* topological mate sorting is a crucial part of robot structure logic ([14ee866](https://github.com/neurobionics/onshape-robotics-toolkit/commit/14ee866ee96545cd18371db755724baac223c17e))
* transformations for both patterned and other entity have correct orientations and minor positional differences due to unknown reason ([5785ece](https://github.com/neurobionics/onshape-robotics-toolkit/commit/5785eced3fc00485f5a7806225162d0a504396ce))
* traverse instances even if they are in sub-assemblies so we have refs for mates ([72c53f5](https://github.com/neurobionics/onshape-robotics-toolkit/commit/72c53f52e5ab75d2dc99282fd3f4b2dbbac4ed19))
* undirected graphs donot preserve edge order during iteration ([d282c84](https://github.com/neurobionics/onshape-robotics-toolkit/commit/d282c84d8ae05a9e0d7b71fc91b3ee13f949c0a9))
* update example scripts and remove redundant ones ([171653f](https://github.com/neurobionics/onshape-robotics-toolkit/commit/171653f62bc987db356d2301b3e8361c6e095347))
* update unit tests; they still need to be made better ([e7af28b](https://github.com/neurobionics/onshape-robotics-toolkit/commit/e7af28bd1683478b57946ecf32886aa5e611ec4f))
* use pypi token instead of trusted publishing ([7296887](https://github.com/neurobionics/onshape-robotics-toolkit/commit/729688761849790cf46f99f6a526d48e99b08011))
* uv version tag ([684ff4d](https://github.com/neurobionics/onshape-robotics-toolkit/commit/684ff4dbd43ad80d5d828566431202aabc13809c))
* windows forward slash issue ([e585f88](https://github.com/neurobionics/onshape-robotics-toolkit/commit/e585f88ed3a41e21b070a303af74496ffc0e45fc))


### Documentation

* add info about mate pattern transforms ([126326a](https://github.com/neurobionics/onshape-robotics-toolkit/commit/126326ae41271133e5b4aad8885b73fffa3d8bfe))
* add info about transformations and assumptions ([98f5834](https://github.com/neurobionics/onshape-robotics-toolkit/commit/98f583429f8093b1f1c82eb802a5e12f3517f484))
* information on how rigid-subassembly -&gt; mate is handled ([2f42102](https://github.com/neurobionics/onshape-robotics-toolkit/commit/2f42102f2c1c16f6dc5624d34f7675297e8519a9))
* update docstrings for the link module ([a608d48](https://github.com/neurobionics/onshape-robotics-toolkit/commit/a608d4860a09837c756da576c7b11c6ca299bc71))
* updated docs to reflect new changes to tutorial scripts ([b754731](https://github.com/neurobionics/onshape-robotics-toolkit/commit/b754731ef91cdef5f6bb3f3c352ceb0fffeded57))
