#include <codac.h>
#include <codac-rob.h>

using namespace std;
using namespace codac;

int main()
{

    // design the true robot trajectory
    double dt = 0.1;
    Interval tdomain1(0, 10);
    Interval tdomain2(tdomain1.ub(), tdomain1.ub()+10);

    TrajectoryVector actual_x1(tdomain1, TFunction("( t; 0; 0; 1 )"), dt);
    // cout << "actual_x1 = " << actual_x1 << endl;

    TrajectoryVector actual_x2(tdomain1, TFunction("( 10; -t; -90; 1 )"), dt);
    // cout << "actual_x2 = " << actual_x2 << endl;
    // actual_x2.shift_tdomain(10);
    // cout << "actual_x1 = " << actual_x1 << end;

    TrajectoryVector actual_x3(tdomain1, TFunction("( 10-t; -10; 180; 1 )"), dt);
    // cout << "actual_x3 = " << actual_x3 << endl;
    // actual_x3.shift_tdomain(20);
    // cout << "actual_x1 = " << actual_x1 << endl;
    // cout << "actual_x2 = " << actual_x2 << endl;
    // cout << "actual_x3 = " << actual_x3 << endl;

    TrajectoryVector actual_x4(tdomain1, TFunction("( 0; -10+t; 90; 1 )"), dt);
    // cout << "actual_x4 = " << actual_x4 << endl;
    // actual_x4.shift_tdomain(30);
    cout << "actual_x1 = " << actual_x1 << endl;
    cout << "actual_x2 = " << actual_x2 << endl;
    cout << "actual_x3 = " << actual_x3 << endl;
    cout << "actual_x4 = " << actual_x4 << endl;

    // container for the 4 trajectories
    vector<TrajectoryVector> actual_x = {actual_x1, actual_x2, actual_x3, actual_x4};

    vibes::beginDrawing();
    VIBesFigMap fig_map("Map");
    fig_map.set_properties(100, 100, 1000, 500);

    // Display the true trajectory
    fig_map.add_trajectory(&actual_x1, "actual_x1", 0, 1, "#276279", 0);
    fig_map.add_trajectory(&actual_x2, "actual_x2", 0, 1, "#276279", 0);
    fig_map.add_trajectory(&actual_x3, "actual_x3", 0, 1, "#276279", 0);
    fig_map.add_trajectory(&actual_x4, "actual_x4", 0, 1, "#276279", 0);

    // draw the corridor
    IntervalVector out_box{{-1, 11}, {-11, 1}};
    IntervalVector in_box{{1, 9}, {-9, -1}};
    fig_map.draw_box(out_box);
    fig_map.draw_box(in_box);

    // Creating random map of landmarks
    int nb_landmarks = 500;
    vector<IntervalVector> v_map =
            DataLoader::generate_landmarks_boxes(out_box, nb_landmarks);
    vector<IntervalVector> l_map;

    // vinicity of true trajectory
    IntervalVector move_area1(actual_x1.codomain().subvector(0, 1));
    move_area1.inflate(0.4);
    cout << "move_area1" << move_area1 << endl;
    IntervalVector move_area2(actual_x2.codomain().subvector(0, 1));
    move_area2.inflate(0.4);
    cout << "move_area2" << move_area2 << endl;
    IntervalVector move_area3(actual_x3.codomain().subvector(0, 1));
    move_area3.inflate(0.4);
    cout << "move_area3" << move_area3 << endl;
    IntervalVector move_area4(actual_x4.codomain().subvector(0, 1));
    move_area4.inflate(0.4);
    cout << "move_area4" << move_area4 << endl;
    
    // draw landmarks(beacons) in the map
    for(auto& b : v_map)
        if(b.is_disjoint(in_box) && b.is_disjoint(move_area1) && b.is_disjoint(move_area2) 
        && b.is_disjoint(move_area3) && b.is_disjoint(move_area4) ){
            fig_map.add_beacon(b.mid(), 0.1); 
            IntervalVector l = b.inflate(0.05);
            l_map.push_back(l);
        } 
    cout << "l_map.size() = " << l_map.size() << endl;

    // define the observation range
    double r_obs = 2; // range of oservation


    // iterate the contractor over whole trajectory
    for(int j = 0; j < 4; j++)
    {
        for(int i=0; i < 100; i++)
        {
            
            // draw robot at actual_x[j](t=dt*i)
            IntervalVector vehicle_i(3);
            vehicle_i[0] = actual_x[j](dt*i)[0];
            vehicle_i[1] = actual_x[j](dt*i)[1];
            if(j == 0)
                vehicle_i[2] = Interval(0);
            if(j == 1)
                vehicle_i[2] = Interval(-M_PI/2);
            if(j == 2)
                vehicle_i[2] = Interval(-M_PI);
            if(j == 3)
                vehicle_i[2] = Interval(M_PI/2);
            Vector vehicle = vehicle_i.mid();
            fig_map.draw_vehicle(vehicle, 0.5);

            // Generating observations for actual_x(t=dt*i) of landmarks
            // Display range-and-bearing measurements
            // cout << "actual_x[j](0).subvector(0, 1): " << actual_x[j](dt*i).subvector(0, 1) << endl;

            vector<IntervalVector> l1_obs; // Define the 2d range-and-bearing measurements
            vector<IntervalVector> l1_map; // define map of the actual_x[j](t=dt*i)

            for(auto& l1_0: l_map)
            {
                Vector l1_d = l1_0.mid() - actual_x[j](dt*i).subvector(0, 1);
                double L1_D = sqrt(l1_d[0] * l1_d[0] + l1_d[1] * l1_d[1]); // landmark to robot distance
                Interval l1_r = L1_D + Interval(-0.1, 0.1); // range-to-landmark observed
                double l1_phi = atan2(l1_d[1], l1_d[0]) - actual_x[j](dt*i)[2]; // landmark to robot angle
                Interval l1_a = l1_phi + Interval(-0.04, 0.04); // angle-to-landmark observed
                if(L1_D < r_obs)
                {
                    l1_obs.push_back({l1_r, l1_a});
                    l1_map.push_back(l1_0);
                    if(j == 0)
                        fig_map.draw_box(l1_0, "grey[blue]");
                    if(j == 1)
                        fig_map.draw_box(l1_0, "grey[red]");
                    if(j == 2)
                        fig_map.draw_box(l1_0, "grey[green]");
                    if(j == 3)
                        fig_map.draw_box(l1_0, "grey[yellow]");
                    // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], l1_r, l1_a);
                    // fig_map.draw_pie(actual_x[j](dt*i)[0], actual_x[j](dt*i)[1], (Interval(0.01) | l1_r), l1_a, "lightGray");
                }
                fig_map.axis_limits(fig_map.view_box(), true, 0.1);        
            }
            fig_map.show();

            // Building contractor
            IntervalVector x(3);
            x[2] = Interval(actual_x[j](dt*i)[2]);
            CtcFunction ctc_plus1(Function("x", "d", "m", "x + d - m")); // x + d = m
            CtcFunction ctc_plus2(Function("x3", "y2", "theta", "x3 + y2 - theta")); // x3 + y2 = theta

            ContractorNetwork cn1;

            for(int i = 0; i < l1_obs.size(); i++)
            {
                IntervalVector& d = cn1.create_interm_var(IntervalVector(2));
                Interval& theta = cn1.create_interm_var(Interval());

                cn1.add(ctc::polar, {d[0], d[1], l1_obs[i][0], theta});
                cn1.add(ctc_plus1, {x[0], d[0], l1_map[i][0]});
                cn1.add(ctc_plus1, {x[1], d[1], l1_map[i][1]});
                cn1.add(ctc_plus2, {x[2], l1_obs[i][1], theta});
            }
            cn1.contract();
            // cout << "x = " << x << endl;

            fig_map.draw_box(x.subvector(0, 1), "red");
            fig_map.show(); // argument is robot size
        }
    }

    vibes::endDrawing();

}

