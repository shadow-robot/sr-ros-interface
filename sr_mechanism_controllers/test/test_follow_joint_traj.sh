rostopic pub -1 /r_arm_joint_trajectory_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal "header:
  seq: 1
  stamp:
    secs: 249
    nsecs: 986000000
  frame_id: ''
goal_id:
  stamp:
    secs: 249
    nsecs: 986000000
  id: /move_group-2-249.986000000
goal:
  trajectory:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: world
    joint_names: ['ElbowJRotate', 'ElbowJSwing', 'ShoulderJRotate', 'ShoulderJSwing', 'WRJ1', 'WRJ2']
    points:
      -
        positions: [1.2494605536561725, 1.3673636863629373, 0.10206458638157478, 4.5683224570325365e-05, 0.20529939420439014, 0.029518588271090884]
        velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        time_from_start:
          secs: 0
          nsecs: 0
      -
        positions: [1.2142727642124294, 1.3598755449420339, 0.120051818234647, 0.027574584959812663, 0.2052827463007976, 0.029504395667823223]
        velocities: [-0.11829358945876972, -0.025173480376473586, 0.060469107436510995, 0.0925460977117055, -5.5966581136729897e-05, -4.771240282029636e-05]
        accelerations: [-0.20816986870016452, -0.04429961191196384, 0.10641190459320732, 0.16286012705762976, -9.848848023071341e-05, -8.396299982027644e-05]
        time_from_start:
          secs: 1
          nsecs: 415938947
      -
        positions: [1.1790849747686862, 1.3523874035211305, 0.13803905008771922, 0.055103486695055, 0.20526609839720505, 0.029490203064555565]
        velocities: [-0.17114093416380438, -0.03641966540617555, 0.0874835194543058, 0.1338908193424662, -8.096950157246435e-05, -6.902779116962787e-05]
        accelerations: [-0.18396591819986174, -0.039148887551157516, 0.0940393720672506, 0.14392434889535655, -8.70372057727959e-05, -7.420060575156271e-05]
        time_from_start:
          secs: 2
          nsecs: 647454726
      -
        positions: [1.143897185324943, 1.3448992621002271, 0.15602628194079143, 0.08263238843029734, 0.2052494504936125, 0.029476010461287904]
        velocities: [-0.19514656876067488, -0.04152818712923764, 0.09975467720817316, 0.15267144655310472, -9.23269496180553e-05, -7.871019672587318e-05]
        accelerations: [-0.053799605999214205, -0.011448831099636734, 0.027501187258691783, 0.04208971607366666, -2.5453450419885748e-05, -2.1699472344634645e-05]
        time_from_start:
          secs: 3
          nsecs: 832368328
      -
        positions: [1.1087093958812, 1.3374111206793238, 0.17401351379386365, 0.11016129016553967, 0.20523280259001997, 0.029461817858020243]
        velocities: [-0.2, -0.04256102210043685, 0.10223564558853264, 0.15646849188554113, -9.462318523394951e-05, -8.066777420247861e-05]
        accelerations: [0.0, 1.4986901565387163e-15, -3.628407747409523e-15, -5.205976333239752e-15, 3.3893075086196296e-18, 2.9271292119896802e-18]
        time_from_start:
          secs: 4
          nsecs: 8307275
      -
        positions: [1.0735216064374566, 1.3299229792584204, 0.19200074564693587, 0.13769019190078202, 0.20521615468642743, 0.02944762525475258]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554163, -9.46231852339498e-05, -8.0667774202469e-05]
        accelerations: [0.0, -3.036819527723198e-15, 7.335693924110581e-15, 1.0727466383645582e-14, -6.778615017239281e-18, 1.0630100822488872e-16]
        time_from_start:
          secs: 5
          nsecs: 184246222
      -
        positions: [1.0383338169937137, 1.322434837837517, 0.20998797750000808, 0.16521909363602436, 0.20519950678283488, 0.029433432651484924]
        velocities: [-0.2, -0.04256102210043762, 0.10223564558853296, 0.15646849188554157, -9.46231852339498e-05, -8.0667774202469e-05]
        accelerations: [0.0, -4.1411175378043605e-15, -7.335693924110581e-15, -1.1200736959394652e-14, 6.778615017239281e-18, -1.0630100822488872e-16]
        time_from_start:
          secs: 6
          nsecs: 360185169
      -
        positions: [1.0031460275499704, 1.3149466964166134, 0.2279752093530803, 0.1927479953712667, 0.20518285887924234, 0.029419240048217263]
        velocities: [-0.2, -0.04256102210043748, 0.10223564558853264, 0.1564684918855411, -9.462318523394951e-05, -8.066777420247861e-05]
        accelerations: [0.0, 5.67924690898882e-15, 3.628407747409523e-15, 5.67924690898882e-15, -3.3893075086196296e-18, -2.9271292119896802e-18]
        time_from_start:
          secs: 7
          nsecs: 536124117
      -
        positions: [0.9679582381062273, 1.30745855499571, 0.24596244120615252, 0.22027689710650902, 0.2051662109756498, 0.0294050474449496]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554168, -9.46231852339498e-05, -8.066777420247886e-05]
        accelerations: [0.0, 0.0, 0.0, 7.887842929151163e-16, 0.0, 0.0]
        time_from_start:
          secs: 8
          nsecs: 712063064
      -
        positions: [0.9327704486624842, 1.2999704135748067, 0.26394967305922473, 0.24780579884175138, 0.20514956307205726, 0.02939085484168194]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554168, -9.462318523387093e-05, -8.0667774202469e-05]
        accelerations: [0.0, 0.0, 0.0, -7.887842929151163e-16, 8.96625895462105e-16, 1.121552666488681e-16]
        time_from_start:
          secs: 9
          nsecs: 888002011
      -
        positions: [0.897582659218741, 1.2924822721539033, 0.28193690491229695, 0.2753347005769937, 0.20513291516846474, 0.029376662238414283]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554152, -9.462318523387093e-05, -8.0667774202469e-05]
        accelerations: [0.0, 0.0, 0.0, -9.465411514981395e-16, -8.96625895462105e-16, -1.121552666488681e-16]
        time_from_start:
          secs: 10
          nsecs: 63940958
      -
        positions: [0.8623948697749979, 1.284994130733, 0.29992413676536916, 0.302863602312236, 0.2051162672648722, 0.02936246963514662]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554157, -9.46231852339498e-05, -8.066777420247886e-05]
        accelerations: [0.0, 0.0, 0.0, 1.7353254444132559e-15, 0.0, 0.0]
        time_from_start:
          secs: 11
          nsecs: 239879905
      -
        positions: [0.8272070803312548, 1.2775059893120966, 0.3179113686184414, 0.3303925040474784, 0.20509961936127966, 0.02934827703187896]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554174, -9.46231852339498e-05, -8.066777420247886e-05]
        accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        time_from_start:
          secs: 12
          nsecs: 415818853
      -
        positions: [0.7920192908875117, 1.2700178478911932, 0.3358986004715136, 0.35792140578272075, 0.2050829714576871, 0.0293340844286113]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554157, -9.46231852339498e-05, -8.0667774202469e-05]
        accelerations: [0.0, 0.0, 0.0, -1.7353254444132559e-15, 0.0, 1.121552666488681e-16]
        time_from_start:
          secs: 13
          nsecs: 591757800
      -
        positions: [0.7568315014437685, 1.2625297064702898, 0.3538858323245858, 0.38545030751796305, 0.20506632355409457, 0.02931989182534364]
        velocities: [-0.2, -0.04256102210043691, 0.10223564558853279, 0.15646849188554118, -9.462318523394966e-05, -8.066777420246888e-05]
        accelerations: [0.0, 7.493450782693593e-16, -1.814203873704765e-15, -2.8396234544944143e-15, 1.6946537543098175e-18, -1.1069170204287308e-16]
        time_from_start:
          secs: 14
          nsecs: 767696747
      -
        positions: [0.7216437120000253, 1.2550415650493865, 0.37187306417765803, 0.41297920925320536, 0.20504967565050203, 0.02930569922207598]
        velocities: [-0.2, -0.04256102210043691, 0.10223564558853279, 0.15646849188554135, -9.462318523394966e-05, -8.066777420247874e-05]
        accelerations: [0.0, -7.493450782693593e-16, 1.814203873704765e-15, 4.5749488989076676e-15, -1.6946537543098175e-18, -1.4635646059948424e-18]
        time_from_start:
          secs: 15
          nsecs: 943635694
      -
        positions: [0.6864559225562822, 1.247553423628483, 0.38986029603073025, 0.4405081109884477, 0.20503302774690949, 0.02929150661880832]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554157, -9.46231852339498e-05, -8.0667774202469e-05]
        accelerations: [0.0, 0.0, 0.0, -1.7353254444132559e-15, 0.0, 1.121552666488681e-16]
        time_from_start:
          secs: 16
          nsecs: 119574642
      -
        positions: [0.6512681331125391, 1.2400652822075797, 0.40784752788380246, 0.46803701272369, 0.20501637984331694, 0.02927731401554066]
        velocities: [-0.2, -0.04256102210043755, 0.10223564558853279, 0.15646849188554152, -9.462318523394966e-05, -8.066777420246888e-05]
        accelerations: [0.0, -6.428591987258188e-15, -1.814203873704765e-15, 7.887842929151151e-16, 1.6946537543098175e-18, -1.1069170204287308e-16]
        time_from_start:
          secs: 17
          nsecs: 295513589
      -
        positions: [0.6160803436687958, 1.2325771407866761, 0.4258347597368747, 0.49556591445893244, 0.2049997319397244, 0.029263121412273]
        velocities: [-0.2, -0.04256102210043755, 0.10223564558853279, 0.15646849188554152, -9.462318523394966e-05, -8.066777420247874e-05]
        accelerations: [0.0, 6.428591987258188e-15, 1.814203873704765e-15, -7.887842929151151e-16, -1.6946537543098175e-18, -1.4635646059948424e-18]
        time_from_start:
          secs: 18
          nsecs: 471452536
      -
        positions: [0.5808925542250527, 1.2250889993657728, 0.4438219915899469, 0.5230948161941747, 0.20498308403613186, 0.02924892880900534]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554157, -9.46231852339498e-05, -8.066777420247886e-05]
        accelerations: [0.0, 0.0, 0.0, 1.7353254444132559e-15, 0.0, 0.0]
        time_from_start:
          secs: 19
          nsecs: 647391483
      -
        positions: [0.5457047647813096, 1.2176008579448694, 0.4618092234430191, 0.5506237179294171, 0.20496643613253931, 0.029234736205737678]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554143, -9.46231852339498e-05, -8.0667774202469e-05]
        accelerations: [0.0, 0.0, 0.0, -3.4706508888265118e-15, 0.0, 1.121552666488681e-16]
        time_from_start:
          secs: 20
          nsecs: 823330430
      -
        positions: [0.5105169753375665, 1.210112716523966, 0.47979645529609133, 0.5781526196646594, 0.20494978822894677, 0.02922054360247002]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554143, -9.46231852339498e-05, -8.0667774202469e-05]
        accelerations: [0.0, 0.0, 0.0, 3.4706508888265118e-15, 0.0, -1.121552666488681e-16]
        time_from_start:
          secs: 21
          nsecs: 999269378
      -
        positions: [0.47532918589382334, 1.2026245751030626, 0.49778368714916355, 0.6056815213999017, 0.20493314032535423, 0.02920635099920236]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853279, 0.15646849188554174, -9.46231852339498e-05, -8.066777420247886e-05]
        accelerations: [0.0, 0.0, -1.8142038737047676e-15, 0.0, 0.0, 0.0]
        time_from_start:
          secs: 22
          nsecs: 175208325
      -
        positions: [0.4401413964500802, 1.1951364336821593, 0.5157709190022357, 0.6332104231351441, 0.2049164924217617, 0.029192158395934698]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554174, -9.46231852339498e-05, -8.066777420247886e-05]
        accelerations: [0.0, 0.0, 3.628407747409535e-15, 0.0, 0.0, 0.0]
        time_from_start:
          secs: 23
          nsecs: 351147272
      -
        positions: [0.4049536070063371, 1.187648292261256, 0.533758150855308, 0.6607393248703864, 0.20489984451816914, 0.029177965792667036]
        velocities: [-0.2, -0.04256102210043691, 0.10223564558853313, 0.1564684918855415, -9.462318523394966e-05, -8.066777420246888e-05]
        accelerations: [0.0, 7.493450782693593e-16, -1.814203873704765e-15, -2.8396234544944143e-15, 1.6946537543098175e-18, 1.1354180153875776e-16]
        time_from_start:
          secs: 24
          nsecs: 527086219
      -
        positions: [0.36976581756259386, 1.1801601508403525, 0.5517453827083802, 0.6882682266056288, 0.2048831966145766, 0.02916377318939938]
        velocities: [-0.2, -0.04256102210043691, 0.10223564558853279, 0.1564684918855415, -9.462318523394966e-05, -8.066777420246888e-05]
        accelerations: [0.0, -7.493450782693593e-16, -1.814203873704765e-15, 2.8396234544944143e-15, -1.6946537543098175e-18, -1.1354180153875776e-16]
        time_from_start:
          secs: 25
          nsecs: 703025166
      -
        positions: [0.33457802811885073, 1.1726720094194492, 0.5697326145614524, 0.7157971283408712, 0.20486654871098406, 0.029149580586131717]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853264, 0.15646849188554143, -9.46231852339498e-05, -8.066777420247886e-05]
        accelerations: [0.0, 0.0, 0.0, -3.4706508888265118e-15, 0.0, 0.0]
        time_from_start:
          secs: 26
          nsecs: 878964114
      -
        positions: [0.2993902386751076, 1.1651838679985458, 0.5877198464145246, 0.7433260300761134, 0.20484990080739152, 0.029135387982864056]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554143, -9.462318523387093e-05, -8.0667774202469e-05]
        accelerations: [0.0, 0.0, 3.628407747409535e-15, 3.4706508888265118e-15, 8.96625895462105e-16, 1.121552666488681e-16]
        time_from_start:
          secs: 27
          nsecs: 54903061
      -
        positions: [0.2642024492313645, 1.1576957265776424, 0.6057070782675968, 0.7708549318113558, 0.204833252903799, 0.0291211953795964]
        velocities: [-0.2, -0.04256102210043691, 0.10223564558853313, 0.1564684918855415, -9.462318523387078e-05, -8.066777420246888e-05]
        accelerations: [0.0, 7.493450782693593e-16, -1.814203873704765e-15, -2.8396234544944143e-15, -8.949312417077936e-16, -1.1069170204287308e-16]
        time_from_start:
          secs: 28
          nsecs: 230842008
      -
        positions: [0.22901465978762126, 1.150207585156739, 0.6236943101206691, 0.7983838335465981, 0.20481660500020646, 0.029107002776328737]
        velocities: [-0.2, -0.04256102210043691, 0.10223564558853279, 0.15646849188554118, -9.462318523394966e-05, -8.066777420247874e-05]
        accelerations: [0.0, -7.493450782693593e-16, -1.814203873704765e-15, -6.310274343320921e-16, -1.6946537543098175e-18, -1.4635646059948424e-18]
        time_from_start:
          secs: 29
          nsecs: 406780955
      -
        positions: [0.19382687034387813, 1.1427194437358357, 0.6416815419737413, 0.8259127352818404, 0.20479995709661392, 0.029092810173061076]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554174, -9.46231852339498e-05, -8.066777420247886e-05]
        accelerations: [0.0, 0.0, 3.628407747409535e-15, 7.099058636236047e-15, 0.0, 0.0]
        time_from_start:
          secs: 30
          nsecs: 582719903
      -
        positions: [0.158639080900135, 1.1352313023149323, 0.6596687738268135, 0.8534416370170829, 0.20478330919302137, 0.029078617569793415]
        velocities: [-0.2, -0.04256102210043698, 0.10223564558853296, 0.15646849188554174, -9.46231852339498e-05, -8.0667774202469e-05]
        accelerations: [0.0, 0.0, -3.628407747409535e-15, -7.099058636236047e-15, 0.0, 1.121552666488681e-16]
        time_from_start:
          secs: 31
          nsecs: 758658850
      -
        positions: [0.12345129145639189, 1.127743160894029, 0.6776560056798857, 0.8809705387523251, 0.20476666128942883, 0.029064424966525757]
        velocities: [-0.2, -0.042561022100437614, 0.10223564558853296, 0.15646849188554143, -9.46231852339498e-05, -8.0667774202469e-05]
        accelerations: [0.0, -7.177937065527559e-15, 3.628407747409535e-15, 3.4706508888265118e-15, 0.0, -1.121552666488681e-16]
        time_from_start:
          secs: 32
          nsecs: 934597797
      -
        positions: [0.08826350201264876, 1.1202550194731253, 0.695643237532958, 0.9084994404875675, 0.2047500133858363, 0.029050232363258096]
        velocities: [-0.2, -0.042561022100437614, 0.10223564558853296, 0.15646849188554143, -9.46231852339498e-05, -8.066777420247886e-05]
        accelerations: [0.0, 7.177937065527559e-15, -3.628407747409535e-15, -3.4706508888265118e-15, 0.0, 0.0]
        time_from_start:
          secs: 33
          nsecs: 110536744
"

