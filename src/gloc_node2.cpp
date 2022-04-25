#include <codac.h>
#include <codac-rob.h>

#include <cmath>
#include <iostream>
#include <iomanip>
#include <random>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <Eigen/Core>


using namespace std;
// using namespace codac;
// using namespace gtsam;

// save all static landmarks as points in a container


vector<vector<double>> c_map = { {9.5265164375305175782,-6.180318385362625122}, {9.374485611915588379,5.4402376413345336915}, {10.376537442207336426,1.9489263296127319336}, {12.805300235748291016,3.1796901226043701172}, 
{10.234171152114868165,-4.9158663153648376464}, {14.996339917182922364,0.61878454685211181641}, {8.6695989370346069336,7.2836458683013916016}, {12.558025240898132325,-0.89807492494583129882}, {14.366699934005737305,-0.69439870119094848632}, 
{4.2442206740379333497,4.5349953174591064454}, {4.4839200377464294434,-4.4389019608497619628}, {9.2340183258056640625,4.3434201478958129883}, {12.707539081573486329,1.9557595252990722657}, {3.0566672682762145997,3.5712475776672363282}, 
{5.0679624676704406739,-3.9011233448982238769}, {2.4397044777870178223,3.9700201749801635743}, {10.152649164199829102,-2.3997858166694641113}, {-0.98237405717372894287,-0.38087153434753417968}, {2.9429163932800292969,2.836495041847229004}, 
{15.304683446884155274,-0.41095358133316040039}, {5.0129396319389343262,4.79106903076171875}, {4.7468285560607910157,-3.3987622261047363281}, {8.7001149654388427735,3.8950449228286743165}, {10.211356282234191895,-3.1247595548629760742}, 
{8.1920230388641357422,4.2289820909500122071}, {12.494368910789489747,1.7944986820220947266}, {7.0470545291900634766,-7.5627148300409317016}, {5.5255033373832702637,6.8105320930480957032}, {9.7976909875869750977,-2.6285750269889831542}, 
{10.717779874801635743,2.459528803825378418}, {3.2740152478218078614,-1.7487241029739379882}, {1.9699509739875793458,-1.8901607394218444824}, {8.2866760492324829102,3.7675434350967407227}, {7.7738162279129028321,-5.4464370310306549072}, 
{9.9318175315856933594,-5.4284938573837280273}, {7.2324402332305908204,-6.6949506103992462158}, {0.6875063776969909668,-1.0340731143951416015}, {4.9624427556991577149,-4.7638733088970184326}, {15.061296582221984864,0.58174431324005126954}, 
{1.9486730098724365235,-2.0564517974853515625}, {8.4971114397048950196,6.6226444244384765625}, {7.1850455999374389649,-8.5687593631446361541}, {9.326454758644104004,-3.9863296151161193847}, {8.1090987920761108399,6.598695516586303711}, 
{-0.6613006293773651123,-1.0325716137886047363}, {6.5132638216018676758,5.57398223876953125}, {9.8454364538192749024,4.9335597753524780274}, {2.6368264555931091309,-0.71549212932586669921}, {11.076375126838684083,-2.913284003734588623}, 
{14.405549049377441407,0.44117510318756103516}, {11.308604001998901368,1.1112842559814453125}, {4.5656029582023620606,-2.6846514344215393066}, {11.330800890922546387,-1.8440358638763427734}, {10.767421126365661622,-2.9600569009780883789}, 
{5.6230279207229614258,6.8713946342468261719}, {5.5356308221817016602,4.3969237804412841797}, {7.3040970563888549805,-7.9250275567173957824}, {4.5064918994903564454,-4.3651031255722045898}, {2.6025231480598449708,-3.3285918831825256347}, 
{9.4955122470855712891,-5.2870354950428009033}, {13.899780988693237305,-1.8804666996002197265}, {8.8686660528182983399,-4.1078379750251770019}, {0.11611901223659515381,0.21316480636596679688}, {11.095091581344604493,4.6229610443115234375}, 
{5.9798352122306823731,7.6569449901580810547}, {3.2231112122535705567,-2.9650527834892272949}, {6.8973910212516784668,6.063180685043334961}, {13.496842384338378907,-1.8682910799980163574}, {12.102008938789367676,0.27365291118621826172}, 
{3.8140200972557067872,1.9996694326400756836}, {5.0730903148651123047,-3.8090457916259765625}, {1.4735263884067535401,2.6893190145492553711}, {2.0018140077590942383,-2.5420619845390319824}, {7.4963481426239013672,-7.9917625263333320617},
{9.9863876104354858399,-3.401643991470336914}, {10.142240047454833985,-2.87407761812210083}, {6.7754653096199035645,-5.7826406657695770263}, {4.741329491138458252,-4.7870148718357086181}, {8.6186809539794921875,3.5341901779174804688}, 
{9.013996720314025879,4.5093147754669189454}, {6.2378503084182739258,-4.4669417738914489746}, {15.016167879104614258,0.27483093738555908204}, {13.127411007881164551,-1.2083668112754821777}, {10.492190003395080567,4.4929436445236206055}, 
{10.236713886260986329,-4.8790757954120635986}, {7.7909984588623046875,6.4673219919204711915}, {7.8582918643951416016,-6.1950058937072753906}, {6.5285282731056213379,-5.5551196038722991943}, {4.8259188532829284668,4.3512940406799316407}, 
{7.4177916049957275391,5.8258674144744873047}, {9.025842428207397461,-5.3454958498477935791}, {11.862706542015075684,1.3184130191802978516}, {10.689063072204589844,5.1123075485229492188}, {13.718411087989807129,-0.88955676555633544921}, 
{8.7138124704360961915,-4.4556705951690673828}, {11.839698553085327149,-1.0560693740844726562}, {9.5120196342468261719,-5.2421189546585083007}, {7.4788783788681030274,5.8812260627746582032}, {9.0141597986221313477,6.0626571178436279297}, 
{3.2116619348526000977,-3.2967776656150817871}, {1.3750174343585968018,-0.10662639141082763671}, {5.0974217057228088379,5.088829636573791504}, {11.437703013420104981,1.5591584444046020508}, {9.1194901466369628907,-5.0646387934684753417}, 
{4.851124107837677002,-5.8713316619396209716}, {9.2071200609207153321,-3.333373725414276123}, {8.8336954116821289063,-4.4482462406158447265}, {6.4871149659156799317,5.226161956787109375}, {8.9123721122741699219,-6.6335940062999725341}, 
{9.7207137346267700196,-3.8709522485733032226}, {6.7775869369506835938,5.9478060007095336915}, {13.071766972541809083,-2.8166531324386596679}, {9.6926717758178710938,-2.814182281494140625}, {8.6865290403366088868,-4.473633885383605957}, 
{6.4795506000518798829,-7.7753143608570098876}, {4.5209704637527465821,-4.3934138417243957519}, {7.9877567291259765625,4.5857180356979370118}, {7.1149691343307495118,5.7359125614166259766}, {9.683718562126159668,4.1362656354904174805}, 
{4.0759018659591674805,-3.6236101984977722167}, {11.284339666366577149,1.4863268136978149415}, {1.4820163846015930176,-2.5503494739532470703}, {3.0738843679428100586,4.4697725772857666016}, {13.670368432998657227,0.17965757846832275391}, 
{1.2393105328083038331,-3.2083248496055603027}, {5.9554297924041748047,7.5236499309539794922}, {14.278002500534057618,0.72965848445892333985}, {13.399080157279968262,-2.5399280190467834472}, {10.909723043441772461,-3.9012596011161804199}, 
{8.3884916305541992188,4.2116260528564453125}, {13.845851421356201172,0.62418651580810546875}, {5.8980273008346557618,6.1978468894958496094}, {2.8510357141494750977,3.6044436693191528321}, {13.210642099380493165,-1.4278514385223388671}, 
{6.120662987232208252,7.0144851207733154297}, {3.43797224760055542,-1.9707911610603332519}, {12.106977462768554688,3.3906093835830688477}, {12.936275720596313477,1.8655697107315063477}, {4.8006803393363952637,6.2598284482955932618}, 
{9.911890864372253418,4.1172573566436767579}, {7.6396843194961547852,5.5470378398895263672}, {7.4805274009704589844,-6.7581863701343536376}, {7.3366655111312866211,-7.6140908002853393554}, {14.319427609443664551,-0.87852001190185546875}, 
{3.6057240366935729981,-3.6547495722770690917}, {3.6112633347511291504,-3.2591671943664550781}, {7.39241790771484375,7.1155722141265869141}, {12.543436169624328614,2.4849089384078979493}, {7.1343711614608764649,6.7514365911483764649}, 
{2.4590858519077301026,-1.5462017655372619628}, {3.8729160428047180176,-2.5254017114639282226}, {9.8198481798171997071,-4.4584874510765075683}, {6.9592164754867553711,8.431112051010131836}, {8.282344818115234375,7.2917107343673706055}, 
{3.0242393612861633301,2.2777587175369262696}, {13.145739078521728516,-0.10518872737884521484}, {0.11804443597793579102,-1.7038464546203613281}, {9.7147452831268310547,-4.934520810842514038}, {11.764471173286437989,-3.4654076099395751953}, 
{7.2451249361038208008,-7.3858164399862289428}, {4.7063599228858947754,2.7380375862121582032}, {3.9556965231895446778,-2.5894925594329833984}, {5.2998426556587219239,-6.5220353007316589355}, {-0.071190237998962402343,0.11417305469512939454}, 
{7.5610687732696533204,-8.290390893816947937}, {12.158379316329956055,1.2462208271026611329}, {6.21152496337890625,5.434042811393737793}, {9.7124342918395996094,-3.8953979015350341796}, {14.842639684677124024,1.0970309972763061524}, 
{2.8359107375144958497,-2.2044062018394470214}, {4.7764428257942199708,-4.3002054691314697265}, {10.458711743354797364,-2.0663673877716064453}, {9.9170643091201782227,5.387521505355834961}, {10.559590697288513184,-1.4877102971076965332}, 
{6.2218986749649047852,6.6676958799362182618}, {6.4981066584587097168,-6.6193361878395080566}, {13.321389436721801758,0.3380044698715209961}, {10.439984560012817383,4.2322211265563964844}, {0.55038487911224365235,-2.1013042330741882324}, 
{5.7259464263916015625,-4.0581141114234924316}, {1.4102557599544525147,-1.1298558712005615234}, {2.7489358186721801758,-3.731783151626586914}, {-0.74921150505542755126,-0.96484851837158203125}, {3.4818735718727111817,-4.3166420459747314453}, 
{3.8809573054313659668,4.2078205347061157227}, {9.1556645631790161133,3.2348256111145019532}, {8.1388884782791137696,6.7529547214508056641}, {10.343452930450439454,-4.4742368459701538085}, {7.1516124010086059571,8.092388749122619629}, 
{5.1275396943092346192,6.2758530378341674805}, {-1.0541730374097824096,0.036685109138488769532}, {8.3886761665344238282,3.9126809835433959961}, {8.8872483968734741211,-5.5283667147159576416}, {4.4659352898597717286,4.0225667953491210938}, 
{9.4006875753402709961,-4.7909247279167175292}, {11.536723732948303223,2.9686002731323242188}, {13.579410433769226075,2.0594344139099121094}, {10.103544354438781739,-3.7581460475921630859}, {0.036833316087722778321,1.9508403539657592774}, 
{6.2453218698501586915,-6.0183153748512268066}, {1.6988491117954254151,-3.2460935711860656738}, {9.4413455724716186524,3.8377583026885986329}, {4.371656179428100586,-3.7245578169822692871}, {4.8277717232704162598,-3.2965266108512878417}, 
{5.0102584958076477051,-5.0041117966175079345}, {11.016643404960632325,3.9711531400680541993}, {5.0620744824409484864,-5.078001558780670166}, {7.0304387807846069336,-8.7660276386886835098}, {8.9349380731582641602,-4.5833150446414947509}, 
{10.980051636695861817,-4.8731309473514556884}, {7.6325496435165405274,-6.9578745514154434204}, {14.065082311630249024,-1.214487612247467041}, {10.064751029014587403,5.6309931278228759766}, {6.8839375972747802735,8.1506474018096923829}, 
{6.0248855948448181153,7.5609380006790161133}, {6.2852197289466857911,-5.3002392053604125976}, {8.1083306074142456055,-6.8455146700143814086}, {6.2035094499588012696,4.6262633800506591797}, {5.3964349031448364258,-5.8164394497871398925}, 
{8.3629344701766967774,6.7570531368255615235}, {3.1497656702995300293,-2.0315415859222412109}, {10.660104870796203614,3.1565190553665161133}, {8.6040489673614501954,-4.2315655946731567382}, {0.73642209172248840333,0.21680402755737304688}, 
{5.9498288035392761231,-6.5716076195240020751}, {1.580623239278793335,0.4024804830551147461}, {7.5034527778625488282,4.860624074935913086}, {9.0391139984130859375,5.9783853292465209961}, {6.7905039191246032715,6.0302313566207885743}, 
{13.126214742660522461,2.5269262790679931641}, {4.2470353841781616211,-3.9239569902420043945}, {11.955317854881286622,-4.0343770980834960937}, {3.3888057470321655274,5.1407625675201416016}, {-1.0284943729639053344,-0.030570745468139648437}, 
{3.5432425141334533692,-2.6705644726753234863}, {5.0489284396171569825,-4.3665472269058227539}, {10.876341342926025391,4.3569234609603881836}, {2.3263519108295440674,-1.9943211078643798828}, {1.2924649417400360108,0.28575932979583740235}, 
{9.6742943525314331055,3.1394033432006835938}, {4.3905260562896728516,-3.1219174861907958984} };

vector<codac::IntervalVector> k_map;


int main(int argc, char** argv)
{

    // design the true robot trajectory
    double dt = 0.1;
    codac::Interval tdomain1(0, 7);

    // trajectory1 (rotated 45): a_clock45 (a-upper left corner, b-upper right, c-bottom right, d-bottom left, clock-clockwise)
    codac::TrajectoryVector actual_x1(tdomain1, codac::TFunction("( t; t; 45; sqrt(2) )"), dt);
    codac::TrajectoryVector actual_x2(tdomain1, codac::TFunction("( 7+t; 7-t; -45; sqrt(2) )"), dt);
    codac::TrajectoryVector actual_x3(tdomain1, codac::TFunction("( 14-t; -t; -135; sqrt(2) )"), dt);
    codac::TrajectoryVector actual_x4(tdomain1, codac::TFunction("( 7-t; -7+t; 135; sqrt(2) )"), dt);
    
    cout << "actual_x1 = " << actual_x1 << endl;
    cout << "actual_x2 = " << actual_x2 << endl;
    cout << "actual_x3 = " << actual_x3 << endl;
    cout << "actual_x4 = " << actual_x4 << endl;

    // container for the 4 trajectories
    vector<codac::TrajectoryVector> actual_x = {actual_x1, actual_x2, actual_x3, actual_x4};

    vibes::beginDrawing();
    codac::VIBesFigMap fig_map("Map");
    fig_map.set_properties(100, 100, 1000, 500);

    // Display the true trajectory
    fig_map.add_trajectory(&actual_x1, "actual_x1", 0, 1, "#276279", 0);
    fig_map.add_trajectory(&actual_x2, "actual_x2", 0, 1, "#276279", 0);
    fig_map.add_trajectory(&actual_x3, "actual_x3", 0, 1, "#276279", 0);
    fig_map.add_trajectory(&actual_x4, "actual_x4", 0, 1, "#276279", 0);

    for(int j = 0; j < 4; j++) 
    {
        for(int i=0; i < 70; i++)
        {
            // draw robot at actual_x[j](t=dt*i)
            codac::IntervalVector vehicle_i(3);
            vehicle_i[0] = actual_x[j](dt*i)[0];
            vehicle_i[1] = actual_x[j](dt*i)[1];
            if(j == 0)
            {
                // trajectory1 (rotated 45): a_clock45
                vehicle_i[2] = codac::Interval(M_PI/4);
                // // scenario1: a_counterclock
                // vehicle_i[2] = codac::Interval(-M_PI/2);
                // // scenario2: b_clock
                // vehicle_i[2] = codac::Interval(-M_PI/2);
                // // scenario3: c_clock
                // vehicle_i[2] = codac::Interval(-M_PI);
                // scenario4: d_clock
                // vehicle_i[2] = codac::Interval(M_PI/2);
                
            }
                
            if(j == 1)
            {
                // trajectory1 (rotated 45): a_clock45
                vehicle_i[2] = codac::Interval(-M_PI/4);
                // // scenario1: a_counterclock
                // vehicle_i[2] = codac::Interval(-M_PI/2);
                // // scenario2: b_clock
                // vehicle_i[2] = codac::Interval(-M_PI/2);
                // // scenario3: c_clock
                // vehicle_i[2] = codac::Interval(-M_PI);
                // scenario4: d_clock
                // vehicle_i[2] = codac::Interval(M_PI/2);

            }
                
            if(j == 2)
            {
                // trajectory1 (rotated 45): a_clock45
                vehicle_i[2] = codac::Interval(-3*M_PI/4);
                // // scenario1: a_counterclock
                // vehicle_i[2] = codac::Interval(-M_PI/2);
                // // scenario2: b_clock
                // vehicle_i[2] = codac::Interval(-M_PI/2);
                // // scenario3: c_clock
                // vehicle_i[2] = codac::Interval(-M_PI);
                // scenario4: d_clock
                // vehicle_i[2] = codac::Interval(M_PI/2);
            }
             
            if(j == 3)
            {
                // trajectory1 (rotated 45): a_clock45
                vehicle_i[2] = codac::Interval(3*M_PI/4);
                // // scenario1: a_counterclock
                // vehicle_i[2] = codac::Interval(-M_PI/2);
                // // scenario2: b_clock
                // vehicle_i[2] = codac::Interval(-M_PI/2);
                // // scenario3: c_clock
                // vehicle_i[2] = codac::Interval(-M_PI);
                // scenario4: d_clock
                // vehicle_i[2] = codac::Interval(M_PI/2);
            }
                
            codac::Vector vehicle = vehicle_i.mid();
            fig_map.draw_vehicle(vehicle, 0.5);
        }
    }


    // outer boundry of landmarks
    codac::IntervalVector out_box{{-2, 16}, {-9, 9}};
    // fig_map.draw_box(out_box);

    // // Creating random map of landmarks
    // int nb_landmarks = 670;
    // vector<codac::IntervalVector> v_map =
    //         codac::DataLoader::generate_landmarks_boxes(out_box, nb_landmarks);
    // vector<codac::IntervalVector> l_map;

    
    // // draw landmarks(beacons) in the map
    // for(auto& b : v_map)
    //     if( (b[1].mid() < b[0].mid()+2) && (b[1].mid() < 16-b[0].mid()) && (b[1].mid() > -b[0].mid()-2) && (b[1].mid() > b[0].mid()-16)
    //     && !((b[1].mid() < b[0].mid()-2) && (b[1].mid() < 12-b[0].mid()) && (b[1].mid() > -b[0].mid()+2) && (b[1].mid() > b[0].mid()-12)) ){
    //         // fig_map.add_beacon(b.mid(), 0.1); 
    //         vector<double> m{b[0].mid(), b[1].mid()};
    //         c_map.push_back(m);
    //         // cout << setprecision(20) << "m[0] = " << m[0] << endl;
    //         // cout << setprecision(20) << "m[1] = " << m[1] << endl;
    //         // cout << setprecision(20) << "b = " << b << endl;
    //         // cout << setprecision(20) << "b.mid = " << b.mid() << endl;
    //         codac::IntervalVector l = b.inflate(0.05);
    //         l_map.push_back(l);
    //     } 
    // cout << "l_map.size() = " << l_map.size() << endl;

    // // print out the point-value landmark map
    // cout << setprecision(20) << "c_map = { ";
    // for( vector<double> n: c_map) {
    //     cout << "{" << n[0] << "," << n[1] << "}" << ", ";
    // } 
    // cout << "};" << endl;
    // // // or print the following
    // // for(int i = 0; i < c_map.size(); i++)
    // // {
    // //     cout << setprecision(20) << "c_map[i] =  (" << c_map[i][0]<< ", " << c_map[i][1] << ")" << endl;
    // // }

    // create IntervalVector map(k_map) according to the static map(b_map) that we know
    for(vector<double> k : c_map)
    {
        codac::Vector lk{k[0], k[1]};
        codac::IntervalVector llk(lk);
        k_map.push_back(llk.inflate(0.3)); // radius = 30cm (3*sigma) the interval bound of landmarks
    }
    int m_size = c_map.size();
    // cout << "size of c_map: " << m_size << endl;
    // draw the landmarks(beacons) in the static map
    for(auto& b : k_map)
    {
        fig_map.add_beacon(b.mid(), 0.1);
    }


    // define the observation range
    double r_obs = 3; // range of oservation
    // double r_obs = 15; // range of oservation


    // create a factor graph
    gtsam::NonlinearFactorGraph graph;

    // array of gtsam Symbols for poses in actual_x(i)
    static gtsam::Symbol x[280];

    // // create a noise model for odometry
    // gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1)); // 20cm std on x,y, 0.1 rad on theta

    // array of gtsam Symbols for all landmarks 
    cout <<"c_map.size(): "<< c_map.size()<< endl;
    static gtsam::Symbol lm[241];
    for(int k = 0; k < 241; k++)
    {
        lm[k] = gtsam::Symbol('l', k);
        
        // // add prior on landmarks
        // gtsam::Point2 m_prior(b_map[k][0], b_map[k][1]);
        // gtsam::noiseModel::Diagonal::shared_ptr mapNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.1)); //10cm on width and height
        // graph.emplace_shared<gtsam::PriorFactor<gtsam::Point2> >(lm[k], m_prior, mapNoise);
    }

    // create a noise model for the landmark measurements
    gtsam::noiseModel::Diagonal::shared_ptr measurementNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.01, 0.1)); // 0.01 rad std on bearing, 10cm on range

    // create initial estimate
    gtsam::Values initialEstimate;

    // initial estimate for landmarks
    for(int k = 0; k < 241; k++)
    {
        initialEstimate.insert(lm[k], gtsam::Point2(c_map[k][0], c_map[k][1]));
    }

    // iterate the contractor over whole trajectory
    for(int j = 0; j < 4; j++) 
    {
        for(int i=0; i < 70; i++)
        {
            //create pose keys and odometry factors
            if(j == 0)
            {
                //create the key for pose x[i]
                x[i] = gtsam::Symbol('x', i);
                if(i == 0)
                {
                    // add a prior on pose x1 at the origin
                    // trajectory1: a_clock45
                    gtsam::Pose2 prior(0.0, 0.0, M_PI_4);
                    // // scenario1: a_counterclock
                    // gtsam::Pose2 prior(0.0, 0.0, -M_PI_2);
                    // // scenario2: b_clock
                    // gtsam::Pose2 prior(10.0, 0.0, -M_PI_2);
                    // // scenario3: c_clock
                    // gtsam::Pose2 prior(10.0, -10.0, -M_PI);
                    // scenario4: d_clock
                    // gtsam::Pose2 prior(0.0, -10.0, M_PI_2);

                    gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.02, 0.02, 0.01)); // 20cm std on x,y, 0.01 rad on theta
                    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose2> >(x[i], prior, priorNoise); // add directly to graph
                }

               
                // initial estimate for robot poses
                // trajectory1: a_clock45
                initialEstimate.insert(x[i], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], 0));
                // // scenario1: a_counterclock
                // initialEstimate.insert(x[i], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI_2));
                // // scenario2: b_clock
                // initialEstimate.insert(x[i], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI_2));
                // // scenario3: c_clock
                // initialEstimate.insert(x[i], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI));
                // scenario4: d_clock
                // initialEstimate.insert(x[i], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], M_PI_2));
            }
            if(j == 1)
            {
                //create the key for pose x[i+100]
                x[i+70] = gtsam::Symbol('x', i+70);

                // initial estimate for robot poses
                // trajectory1: a_clock
                initialEstimate.insert(x[i+70], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI_4));
                // // scenario1: a_counterclock
                // initialEstimate.insert(x[i+100], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], 0));
                // // scenario2: b_clock
                // initialEstimate.insert(x[i+100], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI));
                // // scenario3: c_clock
                // initialEstimate.insert(x[i+100], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], M_PI_2));
                // scenario4: d_clock
                // initialEstimate.insert(x[i+100], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], 0));
            }
            if(j == 2)
            {
                //create the key for pose x[i+140]
                x[i+140] = gtsam::Symbol('x', i+140);

                // initial estimate for robot poses
                // trajectory1: a_clock
                initialEstimate.insert(x[i+140], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -3*M_PI_4));
                // // scenario1: a_counterclock
                // initialEstimate.insert(x[i+200], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], M_PI_2));
                // // scenario2: b_clock
                // initialEstimate.insert(x[i+200], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -3*M_PI_2));
                // // scenario3: c_clock
                // initialEstimate.insert(x[i+200], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], 0));
                // scenario4: d_clock
                // initialEstimate.insert(x[i+200], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI_2));
            }
            if(j == 3)
            {
                //create the key for pose x[i]
                x[i+210] = gtsam::Symbol('x', i+210);

                // initial estimate for robot poses
                // trajectory1: a_clock
                initialEstimate.insert(x[i+210], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], 3*M_PI_4));
                // // scenario1: a_counterclock
                // initialEstimate.insert(x[i+300], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], M_PI));
                // // scenario2: b_clock
                // initialEstimate.insert(x[i+300], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], 0));
                // // scenario3: c_clock
                // initialEstimate.insert(x[i+300], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI_2));
                // scenario4: d_clock
                // initialEstimate.insert(x[i+300], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI));
            }
            

            // Generating observations for actual_x(t=dt*i) of landmarks
            // Display range-and-bearing measurements
            // cout << "actual_x[j](0).subvector(0, 1): " << actual_x[j](dt*i).subvector(0, 1) << endl;

            vector<codac::IntervalVector> l1_obs; // Define the 2d range-and-bearing measurements
            vector<codac::IntervalVector> l1_map; // define map of the actual_x[j](t=dt*i)
            vector<codac::Vector> p_block = {{2, 0}, {7, 5}, {12, 0}, {7, -5}}; // four corner points of corridor wall that can block the FOV 

            int k_index = 0;
            for(auto& l1_0: k_map)
            {
                // index of landmark
                k_index++;
                
                codac::Vector l1_d = l1_0.mid() - actual_x[j](dt*i).subvector(0, 1);
                double L1_D = sqrt(l1_d[0] * l1_d[0] + l1_d[1] * l1_d[1]); // landmark to robot distance

                // define interval of range measurement: +- 0.3m
                // codac::Interval l1_r = L1_D + codac::Interval(-0.3, 0.3); // landmark-to-robot distance observed

                // add sytematic error to range measurement by 1 sigma
                codac::Interval l1_r = L1_D + codac::Interval(-0.3, 0.3) + 0.1;
                // // add sytematic error to range measurement by 1 sigma
                // codac::Interval l1_r = L1_D + codac::Interval(-0.3, 0.3) + 0.2;

                double l1_psi = atan2(l1_d[1], l1_d[0]); // landmark to robot angle
                double l1_phi = atan2(l1_d[1], l1_d[0]) - (actual_x[j](dt*i)[2]); // landmark to robot angle (robot heading included)
                codac::Interval l1_a = l1_psi + codac::Interval(-0.04, 0.04); // landmark-to-robot observed
                // define interval of bearing measurement: +- 0.03 rad
                codac::Interval l1_b = l1_phi + codac::Interval(-0.03, 0.03);
                // codac::Interval l1_b = l1_psi+j*M_PI_2 + codac::Interval(-0.04, 0.04);

                // create the measurement values
                // trajectory1: a_clock45
                gtsam::Rot2 bearing = gtsam::Rot2::fromAngle(l1_psi+j*M_PI_2-M_PI_4);
                // // scenario1: a_counterclock
                // gtsam::Rot2 bearing = gtsam::Rot2::fromAngle(l1_psi+ M_PI_2 - j*M_PI_2);
                // // scenario2: b_clock
                // gtsam::Rot2 bearing = gtsam::Rot2::fromAngle(l1_psi+ M_PI_2 + j*M_PI_2);
                // // scenario3: c_clock
                // gtsam::Rot2 bearing = gtsam::Rot2::fromAngle(l1_psi+ M_PI + j*M_PI_2);
                // scenario4: d_clock
                // gtsam::Rot2 bearing = gtsam::Rot2::fromAngle(l1_psi+ 3*M_PI_2 + j*M_PI_2);


                double range = L1_D;
                // shift my range measurement by 1 sigma
                range = L1_D + 0.1;
                // // shift my range measurement by 2 sigma
                // range = L1_D + 0.2;

                // // add offset to the range measurement according to a uniform distribution in interval(-0.3, 0.3)
                // std::random_device rd;
                // std::mt19937 gen(rd());
                // std::uniform_real_distribution<> dis(-0.3, 0.3);
                // range = L1_D + dis(gen);
                // cout << "uniform offset: " << dis(gen) << endl;


                // observation constraints 

                // trajectory1: a_clock45
                if(j == 0)
                // // scenario2: b_clock
                // if(j == 3)
                // // scenario3: c_clock
                // if(j == 2)
                // scenario4: d_clock
                // if(j == 1)
                {
                    // when 0 <= x <= 1, find all landmarks that can be observed
                    if(actual_x[j](dt*i)[0] <= 1)
                    {
                        codac::Vector p0_b1 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "p0_phi1:" << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2:" << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() < -l1_0[0].mid()+2 | l1_0[1].mid() > l1_0[0].mid()-2)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[blue]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                
                                // add measurement factor
                                // original trajectory: a_clock45
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+70], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    // when 1 < x <= 9, find all landmarks that can be observed
                    if(actual_x[j](dt*i)[0] > 1 && actual_x[j](dt*i)[0] <= 6)
                    {
                        codac::Vector p0_b1 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() > l1_0[0].mid()-2 )
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[blue]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");
                                
                                // add measurement factor
                                // original trajectory: a_clock45
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    // when 9 < x < 10, find all landmarks that can be observed
                    if(actual_x[j](dt*i)[0] > 6)
                    {
                        codac::Vector p0_b1 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() > l1_0[0].mid()-2 | l1_0[1].mid() > -l1_0[0].mid()+12 )
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[blue]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // if(i == 69)
                                // {
                                //     fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                //     fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");
                                //     cout << setprecision(20) << "bearing:" <<  l1_psi+j*M_PI_2 << endl;
                                //     cout << setprecision(20) << "range:" << range << endl;
                                //     cout << "k_index-1: " << k_index-1 << endl;
                                // }
                                
                                // add measurement factor
                                // original trajectory: a_clock45
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);  
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);  
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);  
                                // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);                          
                            }
                    }
                }

                // trajectory1: a_clock
                if(j == 1)
                // // scenario2: b_clock
                // if(j == 0)
                // // scenario3: c_clock
                // if(j == 3)
                // scenario4: d_clock
                // if(j == 2)
                {
                    if(actual_x[j](dt*i)[0] <= 8)
                    {
                        codac::Vector p0_b1 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() > l1_0[0].mid()-2 | l1_0[1].mid() > -l1_0[0].mid()+12)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[red]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // if(i == 9)
                                // {
                                //     fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                //     fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");
                                //     cout << "k_index-1: " << k_index-1 << endl;
                                //     cout << setprecision(20) << "l1_psi:" << l1_psi << endl;
                                //     cout << setprecision(20) << "l1_a:" << l1_a << endl;
                                //     cout << setprecision(20) << "actual_x[j](dt*i)[2]:" << actual_x[j](dt*i)[2] << endl;
                                //     // normalize bearing to -pi to pi
                                //     double bear_angle = std::fmod((l1_psi+j*M_PI_2) + M_PI, 2*M_PI);
                                //     if(bear_angle < 0)
                                //     {
                                //         bear_angle += 2*M_PI;
                                //         cout << "bearing:" << bear_angle<< endl;
                                //     }
                                //     bear_angle = bear_angle - M_PI;
                                //     cout << "bearing:" << bear_angle << endl;
                                //     cout << "bearing:" << l1_psi+j*M_PI_2<< endl;
                                //     cout << "range:" << range << endl;
                                // }

                                // add measurement factor
                                // original trajectory: a_clock45
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+70], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    if(actual_x[j](dt*i)[0] > 8 && actual_x[j](dt*i)[0] <= 13)
                    {
                        codac::Vector p0_b1 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if((l1_psi < p0_phi1 && l1_psi > p0_phi2) | l1_0[1].mid() > -l1_0[0].mid()+12)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[red]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // original trajectory: a_clock45
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+70], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    if(actual_x[j](dt*i)[0] <= 14)
                    {
                        codac::Vector p0_b1 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() > -l1_0[0].mid()+12 | l1_0[1].mid() < l1_0[0].mid()-12 )
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[red]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // trajectory1: a_clock45
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+70], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                }

                // trajectory1: a_clock
                if(j == 2)    
                // // scenario2: b_clock
                // if(j == 1)  
                // // scenario3: c_clock
                // if(j == 0)
                // scenario4: d_clock
                // if(j == 3)          
                {
                    if(actual_x[j](dt*i)[0] > 13)
                    {
                        codac::Vector p0_b1 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[0]: " << actual_x[j](dt*i)[0] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() > -l1_0[0].mid()+12 | l1_0[1].mid() < l1_0[0].mid()-12)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[green]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // trajectory1: a_clock45
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+140], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    if(actual_x[j](dt*i)[0] > 8 && actual_x[j](dt*i)[0] <= 13)
                    {
                        codac::Vector p0_b1 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if( l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() < l1_0[0].mid()-12)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[green]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // trajectory1: a_clock45
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+140], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario 2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                            }
                    }
                    if(actual_x[j](dt*i)[0] <= 8)
                    {
                        codac::Vector p0_b1 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[0]: " << actual_x[j](dt*i)[0] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() < l1_0[0].mid()-12 | l1_0[1].mid() < -l1_0[0].mid()+2 )
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[green]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // trajectory1: a_clock45
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+140], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario 2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                }

                // trajectory1: a_clock45
                if(j == 3)   
                // // scenario2: b_clock
                // if(j == 2)
                // // scenario3: c_clock
                // if(j == 1)
                // scenario4: d_clock
                // if(j == 0)             
                {
                    if(actual_x[j](dt*i)[0] > 6)
                    {
                        codac::Vector p0_b1 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() < l1_0[0].mid()-12 | l1_0[1].mid() < -l1_0[0].mid()+2 )
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[yellow]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // trajectory: a_clock45
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+210], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    if(actual_x[j](dt*i)[0] > 1 && actual_x[j](dt*i)[0] <= 6)
                    {
                        codac::Vector p0_b1 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if( l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() < -l1_0[0].mid()+2)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[yellow]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // trajectory1: a_clock45
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+210], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario 2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    if(actual_x[j](dt*i)[0] <= 1)
                    {
                        codac::Vector p0_b1 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[0]: " << actual_x[j](dt*i)[0] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() > l1_0[0].mid()-2 | l1_0[1].mid() < -l1_0[0].mid()+2)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[yellow]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // trajectory1: a_clock45
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+210], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario 2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                }


                // // scenario1: a_counterclock
                // if(j == 0)
                // {
                //     if(actual_x[j](dt*i)[1] >= -1)
                //     {
                //         codac::Vector p0_b1 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                //         codac::Vector p0_b2 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                //         double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                //         double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                //         // cout << setprecision(20) << "actual_x[j](dt*i)[0]: " << actual_x[j](dt*i)[0] << endl;
                //         // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                //         // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                //         if(L1_D < r_obs)
                //             if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() < 1 | l1_0[1].mid() > -1)
                //             {
                //                 l1_obs.push_back({l1_r, l1_b});
                //                 l1_map.push_back(l1_0);
                //                 // fig_map.draw_box(l1_0, "grey[yellow]");
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");


                //                 // if(i == 0)
                //                 // {
                //                 //     fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 //     fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");
                //                 //     cout << "k_index-1: " << k_index-1 << endl;
                //                 //     cout << setprecision(20) << "l1_psi:" << l1_psi << endl;
                //                 //     cout << setprecision(20) << "l1_a:" << l1_a << endl;
                //                 //     cout << setprecision(20) << "actual_x[j](dt*i)[2]:" << actual_x[j](dt*i)[2] << endl;
                //                 //     // normalize bearing to -pi to pi
                //                 //     double bear_angle = std::fmod((l1_psi+M_PI_2-j*M_PI_2) + M_PI, 2*M_PI);
                //                 //     if(bear_angle < 0)
                //                 //     {
                //                 //         bear_angle += 2*M_PI;
                //                 //         cout << "bearing:" << bear_angle<< endl;
                //                 //     }
                //                 //     bear_angle = bear_angle - M_PI;
                //                 //     cout << "bearing:" << bear_angle << endl;
                //                 //     cout << "bearing:" << l1_psi+M_PI_2-j*M_PI_2<< endl;
                //                 //     cout << "range:" << range << endl;
                //                 // }

                //                 // add measurement factor
                //                 graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                //             }
                //     }
                //     if(actual_x[j](dt*i)[1] >= -9 && actual_x[j](dt*i)[1] < -1)
                //     {
                //         codac::Vector p0_b1 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                //         codac::Vector p0_b2 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                //         double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                //         double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                //         // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                //         // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                //         // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                //         if(L1_D < r_obs)
                //             if( l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() < 1)
                //             {
                //                 l1_obs.push_back({l1_r, l1_b});
                //                 l1_map.push_back(l1_0);
                //                 // fig_map.draw_box(l1_0, "grey[yellow]");
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                //                 // add measurement factor
                //                 graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                //             }
                //     }
                //     if(actual_x[j](dt*i)[1] < -9)
                //     {
                //         codac::Vector p0_b1 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                //         codac::Vector p0_b2 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                //         double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                //         double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                //         // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                //         // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                //         // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                //         if(L1_D < r_obs)
                //             if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() < 1 | l1_0[1].mid() < -9)
                //             {
                //                 l1_obs.push_back({l1_r, l1_b});
                //                 l1_map.push_back(l1_0);
                //                 // fig_map.draw_box(l1_0, "grey[yellow]");
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                //                 // add measurement factor
                //                 graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                //             }
                //     }
                // }

                // if(j == 1)
                // {
                //     if(actual_x[j](dt*i)[0] <= 1)
                //     {
                //         codac::Vector p0_b1 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                //         codac::Vector p0_b2 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                //         double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                //         double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                //         // cout << setprecision(20) << "actual_x[j](dt*i)[0]: " << actual_x[j](dt*i)[0] << endl;
                //         // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                //         // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                //         if(L1_D < r_obs)
                //             if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() < 1 | l1_0[1].mid() < -9)
                //             {
                //                 l1_obs.push_back({l1_r, l1_b});
                //                 l1_map.push_back(l1_0);
                //                 // fig_map.draw_box(l1_0, "grey[green]");
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                //                 // add measurement factor
                //                 graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                //             }
                //     }
                //     if(actual_x[j](dt*i)[0] > 1 && actual_x[j](dt*i)[0] <= 9)
                //     {
                //         codac::Vector p0_b1 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                //         codac::Vector p0_b2 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                //         double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                //         double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                //         // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                //         // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                //         // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                //         if(L1_D < r_obs)
                //             if( l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() < -9)
                //             {
                //                 l1_obs.push_back({l1_r, l1_b});
                //                 l1_map.push_back(l1_0);
                //                 // fig_map.draw_box(l1_0, "grey[green]");
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                //                 // add measurement factor
                //                 graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                //             }
                //     }
                //     if(actual_x[j](dt*i)[0] > 9)
                //     {
                //         codac::Vector p0_b1 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                //         codac::Vector p0_b2 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                //         double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                //         double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                //         // cout << setprecision(20) << "actual_x[j](dt*i)[0]: " << actual_x[j](dt*i)[0] << endl;
                //         // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                //         // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                //         if(L1_D < r_obs)
                //             if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() > 9 | l1_0[1].mid() < -9)
                //             {
                //                 l1_obs.push_back({l1_r, l1_b});
                //                 l1_map.push_back(l1_0);
                //                 // fig_map.draw_box(l1_0, "grey[green]");
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                //                 // add measurement factor
                //                 graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                //             }
                //     }                                        
                // }

                // if(j == 2)
                // {
                //     if(actual_x[j](dt*i)[1] <= -9)
                //     {
                //         codac::Vector p0_b1 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                //         codac::Vector p0_b2 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                //         double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                //         double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                //         // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                //         // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                //         // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                //         if(L1_D < r_obs)
                //             if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() > 9 | l1_0[1].mid() < -9)
                //             {
                //                 l1_obs.push_back({l1_r, l1_b});
                //                 l1_map.push_back(l1_0);
                //                 // fig_map.draw_box(l1_0, "grey[red]");
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                //                 // add measurement factor
                //                 graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                //             }
                //     }
                //     if(actual_x[j](dt*i)[1] > -9 && actual_x[j](dt*i)[1] <= -1)
                //     {
                //         codac::Vector p0_b1 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                //         codac::Vector p0_b2 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                //         double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                //         double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                //         // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                //         // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                //         // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                //         if(L1_D < r_obs)
                //             if((l1_psi < p0_phi1 && l1_psi > p0_phi2) | l1_0[0].mid() > 9)
                //             {
                //                 l1_obs.push_back({l1_r, l1_b});
                //                 l1_map.push_back(l1_0);
                //                 // fig_map.draw_box(l1_0, "grey[red]");
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                //                 // add measurement factor
                //                 graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                //             }
                //     }
                //     if(actual_x[j](dt*i)[1] > -1)
                //     {
                //         codac::Vector p0_b1 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                //         codac::Vector p0_b2 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                //         double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                //         double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                //         // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                //         // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                //         // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                //         if(L1_D < r_obs)
                //             if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() > -1 | l1_0[0].mid() > 9)
                //             {
                //                 l1_obs.push_back({l1_r, l1_b});
                //                 l1_map.push_back(l1_0);
                //                 // fig_map.draw_box(l1_0, "grey[red]");
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                //                 // add measurement factor
                //                 graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                //             }
                //     }                    
                    
                // }

                // if(j == 3)
                // {
                //     // when 9 < x < 10, find all landmarks that can be observed
                //     if(actual_x[j](dt*i)[0] > 9)
                //     {
                //         codac::Vector p0_b1 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                //         codac::Vector p0_b2 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                //         double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                //         double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);

                //         if(L1_D < r_obs)
                //             if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() > 9 | l1_0[1].mid() > -1)
                //             {
                //                 l1_obs.push_back({l1_r, l1_b});
                //                 l1_map.push_back(l1_0);
                //                 // fig_map.draw_box(l1_0, "grey[blue]");
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                //                 // if(i == 99)
                //                 // {
                //                 //     fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 //     fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");
                //                 //     cout << setprecision(20) << "bearing:" <<  l1_psi+j*M_PI_2 << endl;
                //                 //     cout << setprecision(20) << "range:" << range << endl;
                //                 //     cout << "k_index-1: " << k_index-1 << endl;
                //                 // }
                                
                //                 // add measurement factor
                //                 graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                //             }
                //     }
                //     // when 1 < x <= 9, find all landmarks that can be observed
                //     if(actual_x[j](dt*i)[0] > 1 && actual_x[j](dt*i)[0] <= 9)
                //     {
                //         codac::Vector p0_b1 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                //         codac::Vector p0_b2 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                //         double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                //         double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);

                //         if(L1_D < r_obs)
                //             if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() > -1)
                //             {
                //                 l1_obs.push_back({l1_r, l1_b});
                //                 l1_map.push_back(l1_0);
                //                 // fig_map.draw_box(l1_0, "grey[blue]");
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");
                                
                //                 // add measurement factor
                //                 graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                //             }
                //     }
                //     // when 0 <= x <= 1, find all landmarks that can be observed
                //     if(actual_x[j](dt*i)[0] <= 1)
                //     {
                //         codac::Vector p0_b1 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                //         codac::Vector p0_b2 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                //         double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                //         double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                //         // cout << setprecision(20) << "p0_phi1:" << p0_phi1 << endl;
                //         // cout << setprecision(20) << "p0_phi2:" << p0_phi2 << endl;

                //         if(L1_D < r_obs)
                //             if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() < 1 | l1_0[1].mid() > -1)
                //             {
                //                 l1_obs.push_back({l1_r, l1_b});
                //                 l1_map.push_back(l1_0);
                //                 // fig_map.draw_box(l1_0, "grey[blue]");
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                //                 // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                //                 // add measurement factor
                //                 graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                //             }
                //     }

                // }


                
                fig_map.axis_limits(fig_map.view_box(), true, 0.1);        
            }
            fig_map.show();

            // Building contractor
            codac::IntervalVector x(3);
            x[2] = codac::Interval(actual_x[j](dt*i)[2]);
            codac::CtcFunction ctc_plus1(codac::Function("x", "d", "m", "x + d - m")); // x + d = m
            codac::CtcFunction ctc_plus2(codac::Function("x3", "y2", "theta", "x3 + y2 - theta")); // x3 + y2 = theta

            codac::ContractorNetwork cn1;

            for(int i = 0; i < l1_obs.size(); i++)
            {
                codac::IntervalVector& d = cn1.create_interm_var(codac::IntervalVector(2));
                codac::Interval& theta = cn1.create_interm_var(codac::Interval());

                cn1.add(codac::ctc::polar, {d[0], d[1], l1_obs[i][0], theta});
                cn1.add(ctc_plus1, {x[0], d[0], l1_map[i][0]});
                cn1.add(ctc_plus1, {x[1], d[1], l1_map[i][1]});
                cn1.add(ctc_plus2, {x[2], l1_obs[i][1], theta});
            }
            cn1.contract();
            // cout << "x = " << x << endl;

            // draw the final contracted box of robot pose
            fig_map.draw_box(x.subvector(0, 1), "red");

            // // output all pose boxes
            // cout << x[0].lb() <<", "<<x[1].lb()<<", "<<x[0].diam()<<", "<<x[1].diam()<< endl;
            
            fig_map.show(); // argument is robot size
        }
    }

    // // Print factor graph
    // graph.print("Factor Graph:\n");

    // // Print initial estimate
    // initialEstimate.print("Initial Estimate:\n");

    // Optimize using Levenberg-Marquardt optimization
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
    gtsam::Values result = optimizer.optimize();
    // // print result of x poses
    // for(int i = 0; i < 280; i++)
    // {
    //     result.at(x[i]).print();
    // }
    // // print result of landmark positions
    // for(int i = 0; i < 241; i++)
    // {
    //     result.at(lm[i]).print();
    // }
    

    // Calculate and print marginal covariances for all variables
    gtsam::Marginals marginals(graph, result);

    // // print 2x2 covariance matrix of (x, y) from the 3x3 cov matrix of (x, y, heading)
    // for(int i = 0; i < 280; i++)
    // {
    //     // cout << "x[" <<i<<"] covariance: \n"<<marginals.marginalCovariance(x[i]) << endl;
    //     // cout << "[" <<marginals.marginalCovariance(x[i])<< "], "<< endl;
    //     Eigen::MatrixXd Matrix = marginals.marginalCovariance(x[i]);
    //     Eigen::MatrixXd m = Matrix.topLeftCorner(2, 2);
    //     cout << m << endl;
    // }

    // // print covariance matrix of all landmarks
    // for(int i = 0; i < 241; i++)
    // {
    //     // cout << "l[" <<i<<"] covariance: \n"<<marginals.marginalCovariance(lm[i]) << endl;
    //     Eigen::MatrixXd lmMatrix = marginals.marginalCovariance(lm[i]);
    //     cout << lmMatrix << endl;
    // }
        
    // // save the fig
    // fig_map.save_image("map", "svg", ".");

    vibes::endDrawing();
    

    return 0;

}

