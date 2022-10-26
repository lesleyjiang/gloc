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
vector<vector<double>> a_map ={{0.83390408754348754883,-4.5905218124389648437}, {9.2682585716247558594,-3.3597655296325683593}, {2.0941841602325439454,-0.60818529129028320312}, {0.78708112239837646485,-3.8908264636993408203}, {6.9648418426513671875,-0.86170053482055664062}, {0.6277237534523010254,-2.5226573944091796875}, 
{7.2751133441925048829,-9.338719487190246582}, {0.65115749835968017579,-3.8226068019866943359}, {3.5129032135009765625,-9.0090878009796142578}, {0.98506605625152587891,-1.8107748031616210937}, {-0.99044014327228069305,-4.4890184402465820312}, {1.0571861863136291504,-10.753292448818683624}, {10.609657764434814454,-10.954243509098887443}, 
{6.9662659168243408204,-0.950519561767578125}, {0.91567462682723999024,-1.7664196491241455078}, {8.4802877902984619141,-10.884349986910820007}, {0.95280927419662475586,-4.7397649288177490234}, {2.6340316534042358399,0.429317474365234375}, {9.9930908679962158204,0.69844603538513183594}, {8.747926473617553711,0.422130584716796875}, 
{7.4556739330291748047,0.56489777565002441407}, {3.4416533708572387696,-10.413008436560630798}, {0.69662594795227050782,0.8889923095703125}, {0.6527863144874572754,-7.3505649566650390625}, {5.1492273807525634766,-0.8067874908447265625}, {2.8757716417312622071,-0.57150125503540039062}, {0.59788149595260620118,-3.8820753097534179687}, 
{4.183175802230834961,-0.94644474983215332031}, {5.6828224658966064454,0.70145082473754882813}, {1.3795578479766845704,-0.7794189453125}, {10.979327678680419922,-10.495802193880081176}, {7.8274202346801757813,-9.3678861856460571289}, {10.825326681137084961,-10.179195135831832885}, {7.481067657470703125,-0.7583904266357421875}, 
{9.0578653812408447266,-4.1410744190216064453}, {5.5603756904602050782,0.56017851829528808594}, {1.9612672924995422364,-0.73961210250854492187}, {5.3813893795013427735,-9.2311607599258422851}, {1.8292287588119506836,0.47563529014587402344}, {-0.055222272872924804687,-10.641586579382419586}, {10.485480070114135743,-10.526560202240943908}, 
{7.9683842658996582032,-10.782729804515838623}, {0.52526515722274780274,-6.0387855768203735351}, {4.6199567317962646485,-0.70467114448547363281}, {7.8075990676879882813,-10.813461560755968093}, {2.7931245565414428711,0.99373221397399902344}, {-0.78899776935577392578,-3.3780138492584228515}, {-0.45252300798892974853,-8.6336867213249206542}, 
{5.7363944053649902344,-9.239630281925201416}, {0.84905266761779785157,-9.14606553316116333}, {1.6154427528381347657,-9.5177875161170959472}, {0.88841980695724487305,-2.9033944606781005859}, {0.45047971606254577637,-6.283731698989868164}, {0.9559370875358581543,0.8166828155517578125}, {6.7762961387634277344,-9.420150458812713623}, 
{6.8451724052429199219,-9.4948760867118835449}, {10.730668306350708008,-4.5631718635559082031}, {5.9588940143585205079,-9.3133951425552368164}, {10.958264827728271485,-6.1893815994262695312}, {9.0507822036743164063,-6.3265204429626464843}, {1.2270656824111938477,-0.80614805221557617187}, {10.286240577697753907,0.66207671165466308594}, 
{9.4802703857421875,-5.3342593908309936523}, {6.8904137611389160157,-10.425269216299057006}, {0.73319429159164428711,-4.8782298564910888671}, {7.9510471820831298829,-10.867415938526391983}, {9.3738567829132080079,-4.7461435794830322265}, {-0.74275239557027816772,-5.0945872068405151367}, {10.571153402328491211,-3.5375001430511474609}, 
{2.3102443218231201172,-9.5774205327033996582}, {-0.68942371010780334472,-8.3446272015571594238}, {10.715087890625,0.51242661476135253907}, {9.4255318641662597657,-9.5517182648181915283}, {9.0824015140533447266,-3.1060335636138916015}, {9.3285481929779052735,-2.1293075084686279296}, {9.6069872379302978516,-10.574794098734855651}, 
{6.6082096099853515625,-0.82186007499694824218}, {6.8877058029174804688,-0.77272343635559082031}, {10.605323314666748047,-6.9817290306091308593}, {0.42525872588157653809,-5.8497338294982910156}, {8.8986845016479492188,0.75380682945251464844}, {-0.82098761573433876037,-1.3708324432373046875}, {6.8564434051513671875,-10.676482811570167541}, 
{10.991654396057128907,-7.0238368511199951171}, {-0.59354121983051300048,-2.1212093830108642578}, {9.0530295372009277344,-10.610908523201942443}, {4.9203841686248779297,0.68194866180419921875}, {9.3648502826690673829,-3.6543571949005126953}, {3.4823399782180786133,0.45301675796508789063}, {2.3451795578002929688,-10.661216795444488525}, 
{10.77653336524963379,-7.6631664037704467773}, {5.9215798377990722657,-10.721268616616725921}, {9.1423485279083251954,-4.4117929935455322265}, {0.92296022176742553711,-2.4928019046783447265}, {0.93384981155395507813,-4.2448251247406005859}, {0.60834729671478271485,-1.4600627422332763671}, {2.4658213853836059571,0.74827861785888671875}, 
{0.87627011537551879883,-2.3752017021179199218}, {10.645375490188598633,-10.027844786643981933}, {0.59940713644027709961,-10.68680187314748764}, {2.213166952133178711,0.74175548553466796875}, {10.891301870346069336,-9.9803159534931182861}, {4.8685538768768310547,-9.5003512799739837646}, {9.5596208572387695313,-0.81968569755554199218}, 
{-0.4991960376501083374,-6.769956827163696289}, {2.5966870784759521485,0.76609492301940917969}, {-0.26175008714199066162,-10.571772098541259765}, {-0.83001714944839477539,-0.54333448410034179687}, {9.478494405746459961,-3.0017592906951904296}, {9.0090706348419189454,-2.1471424102783203125}, {5.0144202709197998047,-0.49012517929077148437}, 
{2.6703394651412963868,0.54271841049194335938}, {10.970946311950683594,0.47690987586975097657}, {7.3600063323974609375,-10.85907139629125595}, {0.99672329425811767579,-5.273242354393005371}, {0.42041045427322387696,-2.0861289501190185546}, {10.915210723876953125,-3.5651690959930419921}, {6.6803603172302246094,-9.4006524085998535156}, 
{3.2353281974792480469,0.65195512771606445313}, {-0.44315633177757263183,-10.652695439755916595}, {9.5929160118103027344,-3.6847476959228515625}, {-0.83327433466911315917,-0.49187326431274414062}, {10.506237506866455079,-8.9108422398567199707}, {4.0985724925994873047,0.40847277641296386719}, {5.7832829952239990235,-10.483423799276351928}, 
{5.8693726062774658204,-9.4798539280891418457}, {8.7259974479675292969,-9.2710555791854858398}, {8.2595069408416748047,0.79683804512023925782}, {10.95133042335510254,-6.1877837181091308593}, {0.69354176521301269532,-1.138935089111328125}, {6.6676137447357177735,-10.446688979864120483}, {6.4676632881164550782,0.54176211357116699219}, 
{5.2882037162780761719,-9.2452686429023742675}, {0.56437587738037109375,-3.6276924610137939453}, {6.6215162277221679688,-9.4842944145202636718}, {-0.8154767937958240509,-4.3277640342712402343}, {-0.4937619715929031372,-10.867914024740457534}, {10.823918342590332032,-10.790072910487651824}, {10.751688957214355469,0.42941689491271972657}, 
{1.3363975286483764649,0.7005939483642578125}, {10.840233325958251954,-10.336933434009552001}, {5.4145057201385498047,0.88030767440795898438}, {6.1373403072357177735,-0.58395671844482421875}, {7.34722900390625,-9.0708554983139038085}, {0.6297379136085510254,-7.9642540216445922851}, {0.93829137086868286133,-6.4540652036666870117}, 
{3.1931778192520141602,-0.78480195999145507812}, {9.0756573677062988282,-4.4578297138214111328}, {4.8287212848663330079,-10.564963847398757934}, {9.1984889507293701172,-10.967435959726572036}, {9.53014373779296875,-7.8924573659896850585}, {0.99658989906311035157,-6.6766914129257202148}, {9.3675417900085449219,-6.303734898567199707}, 
{10.634421348571777344,-8.2918397188186645507}, {2.1735303401947021485,0.80153727531433105469}, {7.7263152599334716797,-9.2441372871398925781}, {-0.55250672996044158935,-3.7435410022735595703}, {10.579767227172851563,-7.7690528631210327148}, {9.0467002391815185547,-8.8513638973236083984}, {-0.56979066133499145507,-5.4485905170440673828}, 
{1.2499329447746276856,-10.935369681566953659}, {0.42998012900352478028,-9.3224458098411560058}, {2.8977611064910888672,-9.1225266456604003906}, {10.992113590240478516,0.4619045257568359375}, {9.5182354450225830079,-5.4356998205184936523}, {4.0071735382080078125,0.948215484619140625}, {0.82568883895874023438,-6.824133157730102539}, 
{10.90493464469909668,-9.8275908827781677246}, {0.90145909786224365235,-6.7105280160903930664}};

vector<vector<double>> b_map = { {4.1357575654983520508,-0.44424510002136230468}, {-0.87041388824582099914,-8.8564472198486328125}, {10.601058244705200196,-0.70169138908386230468}, {-0.82175752148032188415,-7.664379119873046875}, {9.1409101486206054688,-4.784349679946899414}, {7.5592114925384521485,-10.729257531464099884},
{4.0670608282089233399,-10.800532989203929901}, {10.993034124374389649,-9.1943148970603942871}, {10.571212053298950196,-7.0267318487167358398}, {-0.52344879508018493652,-9.7324979007244110107}, {1.4535294175148010254,-0.51408267021179199218}, {8.7467198371887207032,0.66297650337219238282}, {2.9366581439971923829,-9.2626781463623046875},
{8.1035468578338623047,-9.2378507256507873535}, {9.4116129875183105469,-8.3975073099136352539}, {8.8446583747863769532,0.50087237358093261719}, {-0.73550801724195480346,-0.67879033088684082031}, {-0.70611022412776947021,-5.1015716791152954101}, {6.2409086227416992188,-0.40983629226684570312}, {-0.67251499742269515991,-9.6977573037147521972},
{10.997309207916259766,0.93891930580139160157}, {0.82337051630020141602,-2.8561625480651855468}, {4.3043833971023559571,-10.76501716673374176}, {10.735855340957641602,0.01082134246826171875}, {0.47025081515312194825,-10.495770990848541259}, {-0.72706668078899383544,-0.59107851982116699218}, {8.6162607669830322266,-9.3901088237762451171},
{0.58536541461944580079,-6.1428313255310058593}, {-0.79994567483663558959,-5.2159382104873657226}, {3.0347347259521484375,-10.472460672259330749}, {-0.64052393287420272827,-4.8186848163604736328}, {-0.9176974557340145111,0.093566417694091796875}, {1.6579132080078125,-10.644764147698879241}, {8.5024871826171875,0.41366910934448242188},
{3.0730681419372558594,-0.73106551170349121093}, {7.4633142948150634766,-9.3106714487075805664}, {10.878825664520263672,-0.951320648193359375}, {5.5464975833892822266,-10.921120328828692436}, {2.8327411413192749024,-0.41876840591430664062}, {-0.84998333454132080078,-7.084842085838317871}, {-0.71751714497804641723,-1.7955863475799560546},
{9.5683083534240722657,-10.838691793382167816}, {-0.75981211662292480468,0.085833787918090820313}, {9.0048792362213134766,-3.748281717300415039}, {6.6370322704315185547,-0.84510445594787597656}, {10.474802732467651368,-5.7351205348968505859}, {10.905663490295410157,-3.7310161590576171875}, {0.78071284294128417969,-6.725963592529296875},
{-0.3135204315185546875,-10.549769848585128784}, {0.77766084671020507813,-9.4693770408630371093}, {9.4096024036407470704,-3.5064265727996826171}, {10.723173141479492188,-2.022089242935180664}, {1.2218727469444274903,-10.792655855417251586}, {6.703166961669921875,-9.1081097722053527832}, {2.4225133657455444336,-0.9392299652099609375},
{10.999587297439575196,-9.1678844094276428222}, {4.5543427467346191407,0.72276043891906738282}, {9.224675893783569336,-3.9500253200531005859}, {7.7466499805450439454,-9.5884721279144287109}, {-0.89470272138714790344,-5.2876912355422973632}, {0.84845757484436035157,-0.41312074661254882812}, {3.2748965024948120118,-9.1519554853439331054},
{-0.58100529015064239501,-1.1707603931427001953}, {-0.86823941022157669067,-2.857244729995727539}, {0.98872351646423339844,-9.1409435868263244628}, {1.5552121400833129883,-10.786600746214389801}, {7.9090316295623779297,-0.50548315048217773437}, {10.425554513931274415,-5.3443187475204467773}, {10.906044721603393555,0.53085160255432128907},
{-0.061861366033554077148,0.967693328857421875}, {9.2025880813598632813,-0.91083788871765136718}, {1.4752749800682067872,-0.66565179824829101562}, {5.2319173812866210938,0.50622963905334472657}, {7.4340395927429199219,-0.7256832122802734375}, {1.6838809251785278321,-10.806227698922157287}, {-0.83853440359234809875,-8.0230121016502380371},
{6.0969533920288085938,-10.775094762444496154}, {1.0977926850318908692,-0.89846467971801757812}, {0.72355425357818603516,-7.1698180437088012695}, {6.1754255294799804688,-10.764174818992614746}, {5.5528004169464111329,-10.575873687863349914}, {0.73869943618774414063,-3.7477996349334716796}, {10.973798036575317383,-2.1643478870391845703},
{9.1399338245391845704,-7.438395857810974121}, {0.51484566926956176758,-5.9183340072631835937}, {8.1143980026245117188,-9.5544912219047546386}, {2.8000675439834594727,-10.816480331122875213}, {2.458208918571472168,-10.485648110508918762}, {5.0719397068023681641,-9.4890236854553222656}, {-0.78375977277755737304,-2.2877581119537353515},
{10.686866044998168946,-6.371417403221130371}, {5.796220541000366211,-0.56674838066101074218}, {2.7019255161285400391,-10.663443163037300109}, {10.935185670852661133,0.13764548301696777344}, {1.0504693984985351563,-10.5906306654214859}, {9.0358741283416748047,-8.6461902856826782226}, {5.6269559860229492188,-0.74788546562194824218},
{8.9879949092864990235,0.51427984237670898438}, {1.1213800311088562012,-10.415654212236404418}, {2.514217376708984375,-10.950490808114409446}, {9.5765631198883056641,-5.0839581489562988281}, {-0.43249875307083129882,-2.7288291454315185546}, {10.923717260360717774,-2.5140607357025146484}, {1.3987390398979187012,-10.871013283729553222},
{8.5398592948913574219,0.47534990310668945313}, {6.9383566379547119141,-0.99491906166076660156}, {-0.88304662331938743591,-8.5441029071807861328}, {6.721981048583984375,-0.43761754035949707031}, {0.70981597900390625,-6.4395759105682373046}, {1.7119194865226745606,-9.379550933837890625}, {0.75002032518386840821,-10.864390082657337188},
{-0.90203690901398658752,-1.5925273895263671875}, {-0.61963222920894622802,-3.8084347248077392578}, {6.6380293369293212891,0.40776467323303222657}, {10.543375015258789063,-3.4795215129852294921}, {8.7490200996398925782,-10.714601494371891021}, {-0.98945148661732673645,0.11353635787963867188}, {7.6838991641998291016,-10.841469723731279373},
{3.860178828239440918,-9.4753190279006958007}, {7.0215981006622314454,-10.82483934611082077}, {-0.44997490942478179931,-6.1663879156112670898}, {2.3413883447647094727,0.41028738021850585938}, {10.695685863494873047,-0.8369541168212890625}, {-0.79605586826801300048,-9.0999673008918762207}, {0.68044310808181762696,-2.1121571063995361328},
{3.3405491113662719727,-10.711770132184028625}, {9.0307607650756835938,-2.6575782299041748046}, {-0.64308946579694747924,-5.9354053735733032226}, {2.3609069585800170899,-10.947403797879815101}, {0.95262885093688964844,-6.0919165611267089843}, {-0.54730641841888427734,-6.9888082742691040039}, {3.2994215488433837891,0.84532237052917480469},
{0.99396973848342895508,-4.8876676559448242187}, {0.020025134086608886719,-10.975269775837659835}, {1.4547537565231323243,0.65246653556823730469}, {9.2059018611907958985,-1.4857890605926513671}, {0.52540248632431030274,0.95081114768981933594}, {10.796132802963256836,-5.5541930198669433593}, {10.937156915664672852,-10.183842062950134277},
{-0.53137610852718353271,0.547626495361328125}, {7.1700687408447265625,0.67069196701049804688}, {9.9654297828674316407,-10.418477818369865417}, {2.7313518524169921875,-9.39170151948928833}, {0.15592435002326965333,-10.710601180791854858}, {10.675665855407714844,-4.8795630931854248046}, {0.59246414899826049805,-8.2052735686302185058},
{7.8123912811279296875,-0.81144809722900390625}, {0.64124691486358642579,-3.2856249809265136718}, {4.8591328859329223633,0.83093786239624023438}, {10.569380283355712891,-0.03857421875}, {1.03977203369140625,-9.1057471036911010742}, {0.85218352079391479493,-3.2503077983856201171}, {4.1909635066986083985,-9.5971446931362152099},
{10.976988792419433594,-0.5017852783203125}, {5.1468992233276367188,0.68693327903747558594}, {0.96524274349212646485,0.66687822341918945313}, {1.7288429737091064454,-9.4496515393257141113}, {-0.4882259368896484375,-10.038994908332824707}, {0.5432688593864440918,-5.4063636064529418945}, {0.46719953417778015137,-5.5025075674057006835},
{9.1914365291595458985,-4.5249035358428955078}, {9.923154592514038086,0.73470520973205566407}, {-0.93126743845641613006,-7.4522085189819335937}, {9.0246124267578125,-8.349521338939666748}, {6.1741745471954345704,-9.3759354948997497558}, {4.8629755973815917969,-9.1729630827903747558}, {9.7626175880432128907,-10.511100441217422485},
{9.2680246829986572266,-1.3739995956420898437}, {9.4534125328063964844,-2.7073636054992675781}, {-0.72352111339569091796,-2.6576719284057617187}, {0.98073446750640869141,-10.557164296507835388}, {10.491318702697753907,-3.1562895774841308593}, {-0.49908258020877838134,-9.0634978413581848144}, {9.0833020210266113282,-8.109443068504333496},
{10.771432161331176758,-4.5345852375030517578}, {3.9683684110641479493,-10.443232595920562744}, {10.764108657836914063,-4.7825486660003662109}, {9.1733362674713134766,-1.5760471820831298828}, {9.5247950553894042969,-10.556791245937347412}, {10.94820404052734375,-9.3013203740119934082}, {9.037610769271850586,-10.550878584384918212},
{0.8206335902214050293,-7.0542742013931274414}, {9.5395321846008300782,-8.1798776984214782714}, {-0.42314217984676361083,-2.2871315479278564453}, {-0.75592561811208724975,0.10165309906005859375}, {8.1560766696929931641,-10.807721622288227081}, {-0.55555304884910583496,-1.1128847599029541015} };

vector<codac::IntervalVector> k_map;


int main(int argc, char** argv)
{

    // design the true robot trajectory
    double dt = 0.1;
    codac::Interval tdomain1(0, 10);

    // original trajectory a_clock (a-upper left corner, b-upper right, c-bottom right, d-bottom left, clock-clockwise)
    codac::TrajectoryVector actual_x1(tdomain1, codac::TFunction("( t; 0; 0; 1 )"), dt);
    codac::TrajectoryVector actual_x2(tdomain1, codac::TFunction("( 10; -t; -90; 1 )"), dt);
    codac::TrajectoryVector actual_x3(tdomain1, codac::TFunction("( 10-t; -10; 180; 1 )"), dt);
    codac::TrajectoryVector actual_x4(tdomain1, codac::TFunction("( 0; -10+t; 90; 1 )"), dt);

    // // scenario1: a_counterclock
    // codac::TrajectoryVector actual_x1(tdomain1, codac::TFunction("( 0; -t; -90; 1 )"), dt);
    // codac::TrajectoryVector actual_x2(tdomain1, codac::TFunction("( t; -10; 0; 1 )"), dt);
    // codac::TrajectoryVector actual_x3(tdomain1, codac::TFunction("( 10; -10+t; 90; 1 )"), dt);
    // codac::TrajectoryVector actual_x4(tdomain1, codac::TFunction("( 10-t; 0; 180; 1 )"), dt);

    // // scenario2: b_clock
    // codac::TrajectoryVector actual_x1(tdomain1, codac::TFunction("( 10; -t; -90; 1 )"), dt);
    // codac::TrajectoryVector actual_x2(tdomain1, codac::TFunction("( 10-t; -10; 180; 1 )"), dt);
    // codac::TrajectoryVector actual_x3(tdomain1, codac::TFunction("( 0; -10+t; 90; 1 )"), dt);
    // codac::TrajectoryVector actual_x4(tdomain1, codac::TFunction("( t; 0; 0; 1 )"), dt);

    // // scenario3: c_clock
    // codac::TrajectoryVector actual_x1(tdomain1, codac::TFunction("( 10-t; -10; 180; 1 )"), dt);
    // codac::TrajectoryVector actual_x2(tdomain1, codac::TFunction("( 0; -10+t; 90; 1 )"), dt);
    // codac::TrajectoryVector actual_x3(tdomain1, codac::TFunction("( t; 0; 0; 1 )"), dt);
    // codac::TrajectoryVector actual_x4(tdomain1, codac::TFunction("( 10; -t; -90; 1 )"), dt);

    // // scenario4: d_clock
    // codac::TrajectoryVector actual_x1(tdomain1, codac::TFunction("( 0; -10+t; 90; 1 )"), dt);
    // codac::TrajectoryVector actual_x2(tdomain1, codac::TFunction("( t; 0; 0; 1 )"), dt);
    // codac::TrajectoryVector actual_x3(tdomain1, codac::TFunction("( 10; -t; -90; 1 )"), dt);
    // codac::TrajectoryVector actual_x4(tdomain1, codac::TFunction("( 10-t; -10; 180; 1 )"), dt);
    
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

    // draw the corridor
    codac::IntervalVector out_box{{-1, 11}, {-11, 1}};
    codac::IntervalVector in_box{{1, 9}, {-9, -1}};
    fig_map.draw_box(out_box);
    fig_map.draw_box(in_box);

    // Creating random map of landmarks
    int nb_landmarks = 500;
    vector<codac::IntervalVector> v_map =
            codac::DataLoader::generate_landmarks_boxes(out_box, nb_landmarks);
    vector<codac::IntervalVector> l_map;

    // vinicity of true trajectory
    codac::IntervalVector move_area1(actual_x1.codomain().subvector(0, 1));
    move_area1.inflate(0.4);
    // cout << "move_area1" << move_area1 << endl;
    codac::IntervalVector move_area2(actual_x2.codomain().subvector(0, 1));
    move_area2.inflate(0.4);
    // cout << "move_area2" << move_area2 << endl;
    codac::IntervalVector move_area3(actual_x3.codomain().subvector(0, 1));
    move_area3.inflate(0.4);
    // cout << "move_area3" << move_area3 << endl;
    codac::IntervalVector move_area4(actual_x4.codomain().subvector(0, 1));
    move_area4.inflate(0.4);
    // cout << "move_area4" << move_area4 << endl;
    
    // // draw landmarks(beacons) in the map
    // for(auto& b : v_map)
    //     if(b.is_disjoint(in_box) && b.is_disjoint(move_area1) && b.is_disjoint(move_area2) 
    //     && b.is_disjoint(move_area3) && b.is_disjoint(move_area4) ){
    //         // fig_map.add_beacon(b.mid(), 0.1); 
    //         vector<double> m{b[0].mid(), b[1].mid()};
    //         b_map.push_back(m);
    //         // cout << setprecision(20) << "m[0] = " << m[0] << endl;
    //         // cout << setprecision(20) << "m[1] = " << m[1] << endl;
    //         // cout << setprecision(20) << "b = " << b << endl;
    //         // cout << setprecision(20) << "b.mid = " << b.mid() << endl;
    //         codac::IntervalVector l = b.inflate(0.05);
    //         l_map.push_back(l);
    //     } 
    // cout << "l_map.size() = " << l_map.size() << endl;

    // // print out the point-value landmark map
    // cout << setprecision(20) << "b_map = { ";
    // for( vector<double> n: b_map) {
    //     cout << "{" << n[0] << "," << n[1] << "}" << ", ";
    // } 
    // cout << "};" << endl;
    // // or print the following
    // for(int i = 0; i < b_map.size(); i++)
    // {
    //     cout << setprecision(20) << "b_map[i] =  (" << b_map[i][0]<< ", " << b_map[i][1] << ")" << endl;
    // }

    // create IntervalVector map(k_map) according to the static map(b_map) that we know
    for(vector<double> k : b_map)
    {
        codac::Vector lk{k[0], k[1]};
        codac::IntervalVector llk(lk);
        k_map.push_back(llk.inflate(0.03)); // radius = 3cm (3*sigma) the interval bound of landmarks, equivalent to the mapNoise(map prior uncertainty) 
    }
    int m_size = b_map.size();
    // cout << "size of b_map: " << m_size << endl;
    // draw the landmarks(beacons) in the static map
    for(auto& b : k_map)
    {
        fig_map.add_beacon(b.mid(), 0.1);
    }


    // // define the observation range
    // double r_obs = 2; // range of oservation
    double r_obs = 5; // range of oservation


    // create a factor graph
    gtsam::NonlinearFactorGraph graph;

    // array of gtsam Symbols for poses in actual_x(i)
    static gtsam::Symbol x[400];

    // // create a noise model for odometry
    // gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1)); // 20cm std on x,y, 0.1 rad on theta

    // array of gtsam Symbols for all landmarks 
    cout <<"b_map.size(): "<< b_map.size()<< endl;
    static gtsam::Symbol lm[187];
    for(int k = 0; k < 187; k++)
    {
        lm[k] = gtsam::Symbol('l', k);
        
        // add prior on landmarks
        gtsam::Point2 m_prior(b_map[k][0], b_map[k][1]);
        gtsam::noiseModel::Diagonal::shared_ptr mapNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.01, 0.01)); //1cm on width and height
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Point2> >(lm[k], m_prior, mapNoise);
    }

    // create a noise model for the landmark measurements
    gtsam::noiseModel::Diagonal::shared_ptr measurementNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.01, 0.1)); // 0.01 rad std on bearing, 10cm on range

    // create initial estimate
    gtsam::Values initialEstimate;

    // initial estimate for landmarks
    for(int k = 0; k < 187; k++)
    {
        initialEstimate.insert(lm[k], gtsam::Point2(b_map[k][0], b_map[k][1]));
    }

    // iterate the contractor over whole trajectory
    for(int j = 0; j < 4; j++) 
    {
        for(int i=0; i < 100; i++)
        {
            //create pose keys and odometry factors
            if(j == 0)
            {
                //create the key for pose x[i]
                x[i] = gtsam::Symbol('x', i);
                if(i == 0)
                {
                    // add a prior on pose x1 at the origin
                    // original trajectory: a_clock
                    gtsam::Pose2 prior(0.0, 0.0, 0.0);
                    // // scenario1: a_counterclock
                    // gtsam::Pose2 prior(0.0, 0.0, -M_PI_2);
                    // // scenario2: b_clock
                    // gtsam::Pose2 prior(10.0, 0.0, -M_PI_2);
                    // // scenario3: c_clock
                    // gtsam::Pose2 prior(10.0, -10.0, -M_PI);
                    // // scenario4: d_clock
                    // gtsam::Pose2 prior(0.0, -10.0, M_PI_2);

                    gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.02, 0.02, 0.01)); // 20cm std on x,y, 0.01 rad on theta
                    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose2> >(x[i], prior, priorNoise); // add directly to graph
                }

               
                // initial estimate for robot poses
                // original trajectory: a_clock
                initialEstimate.insert(x[i], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], 0));
                // // scenario1: a_counterclock
                // initialEstimate.insert(x[i], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI_2));
                // // scenario2: b_clock
                // initialEstimate.insert(x[i], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI_2));
                // // scenario3: c_clock
                // initialEstimate.insert(x[i], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI));
                // // scenario4: d_clock
                // initialEstimate.insert(x[i], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], M_PI_2));
            }
            if(j == 1)
            {
                //create the key for pose x[i+100]
                x[i+100] = gtsam::Symbol('x', i+100);

                // initial estimate for robot poses
                // original trajectory: a_clock
                initialEstimate.insert(x[i+100], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI_2));
                // // scenario1: a_counterclock
                // initialEstimate.insert(x[i+100], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], 0));
                // // scenario2: b_clock
                // initialEstimate.insert(x[i+100], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI));
                // // scenario3: c_clock
                // initialEstimate.insert(x[i+100], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], M_PI_2));
                // // scenario4: d_clock
                // initialEstimate.insert(x[i+100], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], 0));
            }
            if(j == 2)
            {
                //create the key for pose x[i+200]
                x[i+200] = gtsam::Symbol('x', i+200);

                // initial estimate for robot poses
                // original trajectory: a_clock
                initialEstimate.insert(x[i+200], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI));
                // // scenario1: a_counterclock
                // initialEstimate.insert(x[i+200], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], M_PI_2));
                // // scenario2: b_clock
                // initialEstimate.insert(x[i+200], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -3*M_PI_2));
                // // scenario3: c_clock
                // initialEstimate.insert(x[i+200], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], 0));
                // // scenario4: d_clock
                // initialEstimate.insert(x[i+200], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI_2));
            }
            if(j == 3)
            {
                //create the key for pose x[i]
                x[i+300] = gtsam::Symbol('x', i+300);

                // initial estimate for robot poses
                // original trajectory: a_clock
                initialEstimate.insert(x[i+300], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -3*M_PI_2));
                // // scenario1: a_counterclock
                // initialEstimate.insert(x[i+300], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], M_PI));
                // // scenario2: b_clock
                // initialEstimate.insert(x[i+300], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], 0));
                // // scenario3: c_clock
                // initialEstimate.insert(x[i+300], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI_2));
                // // scenario4: d_clock
                // initialEstimate.insert(x[i+300], gtsam::Pose2(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], -M_PI));
            }
            
            

            // draw robot at actual_x[j](t=dt*i)
            codac::IntervalVector vehicle_i(3);
            vehicle_i[0] = actual_x[j](dt*i)[0];
            vehicle_i[1] = actual_x[j](dt*i)[1];
            if(j == 0)
            {
                // original trajectory: a_clock
                vehicle_i[2] = codac::Interval(0);
                // // scenario1: a_counterclock
                // vehicle_i[2] = codac::Interval(-M_PI/2);
                // // scenario2: b_clock
                // vehicle_i[2] = codac::Interval(-M_PI/2);
                // // scenario3: c_clock
                // vehicle_i[2] = codac::Interval(-M_PI);
                // // scenario4: d_clock
                // vehicle_i[2] = codac::Interval(M_PI/2);
                
            }
                
            if(j == 1)
            {
                // original trajectory: a_clock
                vehicle_i[2] = codac::Interval(-M_PI/2);
                // // scenario1: a_counterclock
                // vehicle_i[2] = codac::Interval(0);
                // // scenario2: b_clock
                // vehicle_i[2] = codac::Interval(-M_PI);
                // // scenario3: c_clock
                // vehicle_i[2] = codac::Interval(M_PI/2);
                // // scenario4: d_clock
                // vehicle_i[2] = codac::Interval(0);

            }
                
            if(j == 2)
            {
                // original trajectory: a_clock
                vehicle_i[2] = codac::Interval(-M_PI);
                // // scenario1: a_counterclock
                // vehicle_i[2] = codac::Interval(M_PI/2);
                // // scenario2: b_clock
                // vehicle_i[2] = codac::Interval(M_PI/2);
                // // scenario3: c_clock
                // vehicle_i[2] = codac::Interval(0);
                // // scenario4: d_clock
                // vehicle_i[2] = codac::Interval(-M_PI/2);
            }
             
            if(j == 3)
            {
                // original trajectory: a_clock
                vehicle_i[2] = codac::Interval(M_PI/2);
                // // scenario1: a_counterclock
                // vehicle_i[2] = codac::Interval(M_PI);
                // // scenario2: b_clock
                // vehicle_i[2] = codac::Interval(0);
                // // scenario3: c_clock
                // vehicle_i[2] = codac::Interval(-M_PI/2);
                // // scenario4: d_clock
                // vehicle_i[2] = codac::Interval(-M_PI);
            }
                
            codac::Vector vehicle = vehicle_i.mid();
            // fig_map.draw_vehicle(vehicle, 0.5);

            // Generating observations for actual_x(t=dt*i) of landmarks
            // Display range-and-bearing measurements
            // cout << "actual_x[j](0).subvector(0, 1): " << actual_x[j](dt*i).subvector(0, 1) << endl;

            vector<codac::IntervalVector> l1_obs; // Define the 2d range-and-bearing measurements
            vector<codac::IntervalVector> l1_map; // define map of the actual_x[j](t=dt*i)
            vector<codac::Vector> p_block = {{1, -1}, {9, -1}, {9, -9}, {1, -9}}; // four corner points of corridor wall that can block the FOV 

            int k_index = 0;
            for(auto& l1_0: k_map)
            {
                // index of landmark
                k_index++;
                
                // make the actual measured map feature point randomly distributed in the map interval box l1_0 + (-0.03, 0.03)x(-0.03, 0.03)
                std::random_device rd1;
                std::mt19937 gen1(rd1());
                std::uniform_real_distribution<> dis1(-0.03, 0.03);
                double l1_0x = dis1(gen1);
                double l1_0y = dis1(gen1);
                ibex::Vector l1_random(2);
                l1_random[0] = l1_0[0].mid() + l1_0x;
                l1_random[1] = l1_0[1].mid() + l1_0y;
                // std::cout << "l1_random: " << l1_random << std::endl;
                // std::cout << "l1_0.mid(): " << l1_0.mid() << std::endl;


                // codac::Vector l1_d = l1_0.mid() - actual_x[j](dt*i).subvector(0, 1);
                // use actual measurement point
                codac::Vector l1_d = l1_random - actual_x[j](dt*i).subvector(0, 1);
                double L1_D = sqrt(l1_d[0] * l1_d[0] + l1_d[1] * l1_d[1]); // landmark to robot distance

                // // define interval of range measurement: +- 0.3m
                // codac::Interval l1_r = L1_D + codac::Interval(-0.3, 0.3); // landmark-to-robot distance observed

                // // add sytematic error to range measurement by +1 sigma
                // codac::Interval l1_r = L1_D + codac::Interval(-0.3, 0.3) + 0.1;
                // // add sytematic error to range measurement by -1 sigma
                // codac::Interval l1_r = L1_D + codac::Interval(-0.3, 0.3) - 0.1;
                // // add sytematic error to range measurement by +1 sigma
                // codac::Interval l1_r = L1_D + codac::Interval(-0.3, 0.3) + 0.2;
                // add sytematic error to range measurement by -1 sigma
                codac::Interval l1_r = L1_D + codac::Interval(-0.3, 0.3) - 0.2;

                double l1_psi = atan2(l1_d[1], l1_d[0]); // landmark to robot angle
                double l1_phi = atan2(l1_d[1], l1_d[0]) - (actual_x[j](dt*i)[2]); // landmark to robot angle (robot heading included)
                codac::Interval l1_a = l1_psi + codac::Interval(-0.04, 0.04); // landmark-to-robot observed
                // define interval of bearing measurement: +- 0.03 rad
                codac::Interval l1_b = l1_phi + codac::Interval(-0.03, 0.03);
                // codac::Interval l1_b = l1_psi+j*M_PI_2 + codac::Interval(-0.04, 0.04);

                // create the measurement values
                // original trajectory: a_clock
                gtsam::Rot2 bearing = gtsam::Rot2::fromAngle(l1_psi+j*M_PI_2);
                // // scenario1: a_counterclock
                // gtsam::Rot2 bearing = gtsam::Rot2::fromAngle(l1_psi+ M_PI_2 - j*M_PI_2);
                // // scenario2: b_clock
                // gtsam::Rot2 bearing = gtsam::Rot2::fromAngle(l1_psi+ M_PI_2 + j*M_PI_2);
                // // scenario3: c_clock
                // gtsam::Rot2 bearing = gtsam::Rot2::fromAngle(l1_psi+ M_PI + j*M_PI_2);
                // // scenario4: d_clock
                // gtsam::Rot2 bearing = gtsam::Rot2::fromAngle(l1_psi+ 3*M_PI_2 + j*M_PI_2);


                double range = L1_D;
                // // shift my range measurement by +1 sigma
                // range = L1_D + 0.1;
                // // shift my range measurement by -1 sigma
                // range = L1_D - 0.1;
                // // shift my range measurement by +2 sigma
                // range = L1_D + 0.2;
                // shift my range measurement by -2 sigma
                range = L1_D - 0.2;

                // // add offset to the range measurement according to a uniform distribution in interval(-0.3, 0.3)
                // std::random_device rd;
                // std::mt19937 gen(rd());
                // std::uniform_real_distribution<> dis(-0.3, 0.3);
                // range = L1_D + dis(gen);
                // cout << "uniform offset: " << dis(gen) << endl;


                // observation constraints 

                // original trajectory: a_clock
                if(j == 0)
                // // scenario2: b_clock
                // if(j == 3)
                // // scenario3: c_clock
                // if(j == 2)
                // // scenario4: d_clock
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
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() < 1 | l1_0[1].mid() > -1)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[blue]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // original trajectory: a_clock
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    // when 1 < x <= 9, find all landmarks that can be observed
                    if(actual_x[j](dt*i)[0] > 1 && actual_x[j](dt*i)[0] <= 9)
                    {
                        codac::Vector p0_b1 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() > -1)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[blue]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");
                                
                                // add measurement factor
                                // original trajectory: a_clock
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    // when 9 < x < 10, find all landmarks that can be observed
                    if(actual_x[j](dt*i)[0] > 9)
                    {
                        codac::Vector p0_b1 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() > 9 | l1_0[1].mid() > -1)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[blue]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // if(i == 99)
                                // {
                                //     fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                //     fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");
                                //     cout << setprecision(20) << "bearing:" <<  l1_psi+j*M_PI_2 << endl;
                                //     cout << setprecision(20) << "range:" << range << endl;
                                //     cout << "k_index-1: " << k_index-1 << endl;
                                // }
                                
                                // add measurement factor
                                // original trajectory: a_clock
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);  
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);  
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);  
                                // // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);                          
                            }
                    }
                }

                // original trajectory: a_clock
                if(j == 1)
                // // scenario2: b_clock
                // if(j == 0)
                // // scenario3: c_clock
                // if(j == 3)
                // // scenario4: d_clock
                // if(j == 2)
                {
                    if(actual_x[j](dt*i)[1] > -1)
                    {
                        codac::Vector p0_b1 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() > -1 | l1_0[0].mid() > 9)
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
                                // original trajectory: a_clock
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    if(actual_x[j](dt*i)[1] > -9 && actual_x[j](dt*i)[1] <= -1)
                    {
                        codac::Vector p0_b1 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if((l1_psi < p0_phi1 && l1_psi > p0_phi2) | l1_0[0].mid() > 9)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[red]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // original trajectory: a_clock
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    if(actual_x[j](dt*i)[1] <= -9)
                    {
                        codac::Vector p0_b1 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() > 9 | l1_0[1].mid() < -9)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[red]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // original trajectory: a_clock
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                }

                // original trajectory: a_clock
                if(j == 2)    
                // // scenario2: b_clock
                // if(j == 1)  
                // // scenario3: c_clock
                // if(j == 0)
                // // scenario4: d_clock
                // if(j == 3)          
                {
                    if(actual_x[j](dt*i)[0] > 9)
                    {
                        codac::Vector p0_b1 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[0]: " << actual_x[j](dt*i)[0] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() > 9 | l1_0[1].mid() < -9)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[green]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // original trajectory: a_clock
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    if(actual_x[j](dt*i)[0] > 1 && actual_x[j](dt*i)[0] <= 9)
                    {
                        codac::Vector p0_b1 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if( l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[1].mid() < -9)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[green]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // original trajectory: a_clock
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario 2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                            }
                    }
                    if(actual_x[j](dt*i)[0] <= 1)
                    {
                        codac::Vector p0_b1 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[0]: " << actual_x[j](dt*i)[0] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() < 1 | l1_0[1].mid() < -9)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[green]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // original trajectory: a_clock
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario 2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                }

                // original trajectory: a_clock
                if(j == 3)   
                // // scenario2: b_clock
                // if(j == 2)
                // // scenario3: c_clock
                // if(j == 1)
                // // scenario4: d_clock
                // if(j == 0)             
                {
                    if(actual_x[j](dt*i)[1] < -9)
                    {
                        codac::Vector p0_b1 = p_block[2] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() < 1 | l1_0[1].mid() < -9)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[yellow]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // original trajectory: a_clock
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    if(actual_x[j](dt*i)[1] >= -9 && actual_x[j](dt*i)[1] < -1)
                    {
                        codac::Vector p0_b1 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[0] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[1]: " << actual_x[j](dt*i)[1] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if( l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() < 1)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[yellow]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // original trajectory: a_clock
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario 2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario4: d_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i], lm[k_index-1], bearing, range, measurementNoise);
                                
                            }
                    }
                    if(actual_x[j](dt*i)[1] >= -1)
                    {
                        codac::Vector p0_b1 = p_block[3] - actual_x[j](dt*i).subvector(0, 1);
                        codac::Vector p0_b2 = p_block[1] - actual_x[j](dt*i).subvector(0, 1);
                        double p0_phi1 = atan2(p0_b1[1], p0_b1[0]);
                        double p0_phi2 = atan2(p0_b2[1], p0_b2[0]);
                        // cout << setprecision(20) << "actual_x[j](dt*i)[0]: " << actual_x[j](dt*i)[0] << endl;
                        // cout << setprecision(20) << "p0_phi1: " << p0_phi1 << endl;
                        // cout << setprecision(20) << "p0_phi2: " << p0_phi2 << endl;

                        if(L1_D < r_obs)
                            if(l1_psi < p0_phi1 | l1_psi > p0_phi2 | l1_0[0].mid() < 1 | l1_0[1].mid() > -1)
                            {
                                l1_obs.push_back({l1_r, l1_b});
                                l1_map.push_back(l1_0);
                                // fig_map.draw_box(l1_0, "grey[yellow]");
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                                // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (codac::Interval(0.01) | l1_r), l1_a, "lightGray");

                                // add measurement factor
                                // original trajectory: a_clock
                                graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+300], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario 2: b_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+200], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario3: c_clock
                                // graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> >(x[i+100], lm[k_index-1], bearing, range, measurementNoise);
                                // // scenario4: d_clock
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
    // for(int i = 0; i < 400; i++)
    // {
    //     result.at(x[i]).print();
    // }
    // // print result of landmark positions
    // for(int i = 0; i < 187; i++)
    // {
    //     result.at(lm[i]).print();
    // }
    

    // Calculate and print marginal covariances for all variables
    gtsam::Marginals marginals(graph, result);

    // // // print 2x2 covariance matrix of (x, y) from the 3x3 cov matrix of (x, y, heading)
    // for(int i = 0; i < 400; i++)
    // {
    //     // cout << "x[" <<i<<"] covariance: \n"<<marginals.marginalCovariance(x[i]) << endl;
    //     // cout << "[" <<marginals.marginalCovariance(x[i])<< "], "<< endl;
    //     Eigen::MatrixXd Matrix = marginals.marginalCovariance(x[i]);
    //     Eigen::MatrixXd m = Matrix.topLeftCorner(2, 2);
    //     cout << m << endl;
    // }

    // // print covariance matrix of all landmarks
    // for(int i = 0; i < 187; i++)
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

