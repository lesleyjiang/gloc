#include <codac.h>
#include <codac-rob.h>

using namespace std;
using namespace codac;

int main()
{

    // design the true robot trajectory
    double dt = 0.05;
    Interval tdomain1(0, 3);

    TrajectoryVector actual_x1(tdomain1, TFunction("( 10*cos(t); 10*sin(t); atan2(10*cos(t),(-10*sin(t)))+[-0.01, 0.01]; sqrt(sqr(10*cos(t))+sqr(-10*sin(t)))+[-0.01, 0.01] )"), dt);
    // cout << "actual_x1 = " << actual_x1 << endl;

    vibes::beginDrawing();
    VIBesFigMap fig_map("Map");
    fig_map.set_properties(100, 100, 1000, 500);

    // Display the true trajectory
    fig_map.add_trajectory(&actual_x1, "actual_x1", 0, 1);
   
    // draw the circle


    // Creating random map of landmarks
    int nb_landmarks = 500;
    IntervalVector map_area(actual_x1.codomain().subvector(0, 1));
    map_area.inflate(2);
    vector<IntervalVector> v_map =
            DataLoader::generate_landmarks_boxes(map_area, nb_landmarks);

    // vinicity of true trajectory
    IntervalVector move_area1(actual_x1.codomain().subvector(0, 1));
    move_area1.inflate(0.4);
    cout << "move_area1" << move_area1 << endl;
    
    // draw landmarks(beacons) in the map
    for(auto& b : v_map){
            fig_map.add_beacon(b.mid(), 0.1);
            IntervalVector l = b.inflate(0.1);
        } 
    cout << "l_map.size()" << v_map.size() << endl;

    //Generating observations obs=(t, range, bearing) of these landmarks
    int max_nb_obs = 100;
    Interval visi_range(0, 4); // [0m, 75m]
    Interval visi_angle(-M_PI, M_PI); // lidar
    vector<IntervalVector> v_obs1 =
            DataLoader::generate_observations(actual_x1, v_map, max_nb_obs,
                                              true, visi_range, visi_angle);

    // Adding uncertainties on the observation measurements
    for(auto& obs : v_obs1)
    {
        obs[1].inflate(0.1);
        obs[2].inflate(0.04);
        cout << "obs" << obs << endl;
    }

    // Display range-and-bearing measurements
    // Method 1:
    cout << "Display range-and-bearing measurements\n Using Method 1: " << endl;
    fig_map.add_observations(v_obs1, &actual_x1);



//     // Method 2: only if we know the actual trajectory
//     //    cout << "Display range-and-bearing measurements\n Using Method 2: " << endl;
//     //    for(auto& obs : v_obs)
//     //    {
//     //        Interval x0 = actual_x[0](obs[0]);
//     //        Interval x1 = actual_x[1](obs[0]);
//     //        Interval x2 = actual_x[2](obs[0]);
//     //        fig_map.draw_pie(x0.mid(), x1.mid(), obs[1], (obs[2] + x2));
//     //        fig_map.draw_pie(x0.mid(), x1.mid(), (Interval(0.01) | obs[1]), (obs[2] + x2));
//     //    }
//     //    fig_map.axis_limits(fig_map.view_box(), true, 0.1);

//     // Display the landmarks
//     for(auto& map : v_map)
//     {
//         map.inflate(0.1);
//         fig_map.draw_box(map, "red[orange]");
//     }

//     fig_map.axis_limits(-15, 20, -8, 13, true);
//     fig_map.show(1);

    // define initial domains of variables
    TubeVector x(tdomain1, dt, 4);
    x[2] = Tube(actual_x1[2], dt);
    x[3] = Tube(actual_x1[3], dt);
    cout << "x = " << x << endl;

    TubeVector u(tdomain1, dt, 2);
    cout << "u = " << u << endl;
    TubeVector v({ x[3]*cos(x[2]), x[3]*sin(x[2]), u[0], u[1] } );
    cout << "v = " << v << endl;

    vector<IntervalVector> y; // Define the 2d range-and-bearing measurements
    for(auto& obs : v_obs1)
        y.push_back({obs[1], obs[2]});

    vector<IntervalVector> m(y.size(), IntervalVector(2)); // Association set

    // contractors
    CtcFunction ctc_plus1(Function("x", "d", "m", "x + d - m")); // x + d = m
    CtcFunction ctc_plus2(Function("x3", "y2", "theta", "x3 + y2 - theta")); // x3 + y2 = theta
    CtcConstell ctc_constell(v_map);

    ContractorNetwork cn;
    for(int i = 0; i < y.size(); i++)
    {
        IntervalVector& d = cn.create_interm_var(IntervalVector(2));
        Interval& theta = cn.create_interm_var(Interval());
        IntervalVector& p = cn.create_interm_var(IntervalVector(4));

        cn.add(ctc::polar, {d[0], d[1], y[i][0], theta});
        cn.add(ctc_plus1, {cn.subvector(p, 0, 1), d, m[i]});
        cn.add(ctc_plus2, {x[2](v_obs1[i][0]), y[i][1], theta});
        cn.add(ctc::eval, {v_obs1[i][0], p, x, v});
        cn.add(ctc::deriv, {x, v});
        cn.add(ctc_constell, {m[i]});
    }
    cn.contract(true);

    fig_map.add_tube(&x, "x", 0, 1);
    fig_map.smooth_tube_drawing(1);

    fig_map.show(0.7); // argument is robot size
    vibes::endDrawing();

    

}

