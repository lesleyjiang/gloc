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
    // TrajectoryVector actual_x1(tdomain1, TFunction("( 10*cos(t); 10*sin(t); atan2(10*cos(t),(-10*sin(t)))+[-0.01, 0.01]; sqrt(sqr(10*cos(t))+sqr(-10*sin(t)))+[-0.01, 0.01] )"), dt);
    // cout << "actual_x1 = " << actual_x1 << endl;

    TrajectoryVector actual_x2(tdomain1, TFunction("( 10; -t; -90; 1 )"), dt);
    // cout << "actual_x2 = " << actual_x2 << endl;
    actual_x2.shift_tdomain(10);
    // cout << "actual_x1 = " << actual_x1 << end;

    TrajectoryVector actual_x3(tdomain1, TFunction("( 10-t; -10; 180; 1 )"), dt);
    // cout << "actual_x3 = " << actual_x3 << endl;
    actual_x3.shift_tdomain(20);
    // cout << "actual_x1 = " << actual_x1 << endl;
    // cout << "actual_x2 = " << actual_x2 << endl;
    // cout << "actual_x3 = " << actual_x3 << endl;

    TrajectoryVector actual_x4(tdomain1, TFunction("( 0; -10+t; 90; 1 )"), dt);
    // cout << "actual_x4 = " << actual_x4 << endl;
    actual_x4.shift_tdomain(30);
    cout << "actual_x1 = " << actual_x1 << endl;
    cout << "actual_x2 = " << actual_x2 << endl;
    cout << "actual_x3 = " << actual_x3 << endl;
    cout << "actual_x4 = " << actual_x4 << endl;

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


    // // draw robot at actual_x1(t=0)
    // fig_map.draw_vehicle(actual_x1(0).subvector(0, 2), 0.5);

    // // Generating observations for actual_x1(t=0) of landmarks
    // // Display range-and-bearing measurements
    // cout << "actual_x1(0).subvector(0, 1): " << actual_x1(0).subvector(0, 1) << endl;

    // vector<IntervalVector> l1_obs; // Define the 2d range-and-bearing measurements
    // vector<IntervalVector> l1_map; // define map of the actual_x1(t=0)
    // double r_obs = 10; // range of oservation

    // for(auto& l1_0: l_map)
    // {
    //     Vector l1_d = l1_0.mid() - actual_x1(0).subvector(0, 1);
    //     double L1_D = sqrt(l1_d[0] * l1_d[0] + l1_d[1] * l1_d[1]); // landmark to robot distance
    //     Interval l1_r = L1_D + Interval(-0.1, 0.1); // range-to-landmark observed
    //     double l1_phi = atan2(l1_d[1], l1_d[0]) - actual_x1(0)[2]; // landmark to robot angle
    //     Interval l1_a = l1_phi + Interval(-0.04, 0.04); // angle-to-landmark observed
    //     if(L1_D < r_obs) // landmarks within range of observation 
    //     {
    //         l1_obs.push_back({l1_r, l1_a});
    //         l1_map.push_back(l1_0);
    //         fig_map.draw_box(l1_0, "blue[blue]");
    //         fig_map.draw_pie(actual_x1(0)[0], actual_x1(0)[1], l1_r, l1_a);
    //         // fig_map.draw_pie(actual_x1(0)[0], actual_x1(0)[1], (Interval(0.01) | l1_r), l1_a, "lightGray");
    //     }
    //     fig_map.axis_limits(fig_map.view_box(), true, 0.1);        
    // }
    // cout << "l1_map.size() = " << l1_map.size() << endl;

    // fig_map.show();


    // // Building contractor
    // IntervalVector x(3);
    // x[2] = Interval(actual_x1(0)[2]).inflate(0.04); // uncertainty of robot heading
    // CtcFunction ctc_plus1(Function("x", "d", "m", "x + d - m")); // x + d = m
    // CtcFunction ctc_plus2(Function("x3", "y2", "theta", "x3 + y2 - theta")); // x3 + y2 = theta

    // ContractorNetwork cn1;

    // for(int i = 0; i < l1_obs.size(); i++)
    // {
    //     IntervalVector& d = cn1.create_interm_var(IntervalVector(2));
    //     Interval& theta = cn1.create_interm_var(Interval());

    //     cn1.add(ctc::polar, {d[0], d[1], l1_obs[i][0], theta});
    //     cn1.add(ctc_plus1, {x[0], d[0], l1_map[i][0]});
    //     cn1.add(ctc_plus1, {x[1], d[1], l1_map[i][1]});
    //     cn1.add(ctc_plus2, {x[2], l1_obs[i][1], theta});
    // }
    // cn1.contract();
    // cout << "x = " << x << endl;

    // fig_map.draw_box(x.subvector(0, 1), "red");
    // fig_map.show(); // argument is robot size



    // iterate the contractor over whole trajectory
    for(int i=0; i < 100; i++)
    {
        
        // draw robot at actual_x1(t=dt*i)
        fig_map.draw_vehicle(actual_x1(dt*i).subvector(0, 2), 0.5);

        // Generating observations for actual_x1(t=dt*i) of landmarks
        // Display range-and-bearing measurements
        // cout << "actual_x1(0).subvector(0, 1): " << actual_x1(dt*i).subvector(0, 1) << endl;

        vector<IntervalVector> l1_obs; // Define the 2d range-and-bearing measurements
        vector<IntervalVector> l1_map; // define map of the actual_x1(t=dt*i)

        for(auto& l1_0: l_map)
        {
            Vector l1_d = l1_0.mid() - actual_x1(dt*i).subvector(0, 1);
            double L1_D = sqrt(l1_d[0] * l1_d[0] + l1_d[1] * l1_d[1]); // landmark to robot distance
            Interval l1_r = L1_D + Interval(-0.1, 0.1); // range-to-landmark observed
            double l1_phi = atan2(l1_d[1], l1_d[0]) - actual_x1(dt*i)[2]; // landmark to robot angle
            Interval l1_a = l1_phi + Interval(-0.04, 0.04); // angle-to-landmark observed
            if(L1_D < r_obs)
            {
                l1_obs.push_back({l1_r, l1_a});
                l1_map.push_back(l1_0);
                fig_map.draw_box(l1_0, "blue[blue]");
                // fig_map.draw_pie(actual_x1(dt*i)[0], actual_x1(dt*i)[1], l1_r, l1_a);
                // fig_map.draw_pie(actual_x1(dt*i)[0], actual_x1(dt*i)[1], (Interval(0.01) | l1_r), l1_a, "lightGray");
            }
            fig_map.axis_limits(fig_map.view_box(), true, 0.1);        
        }
        fig_map.show();

        // Building contractor
        IntervalVector x(3);
        x[2] = Interval(actual_x1(dt*i)[2]);
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
        cout << "x = " << x << endl;

        fig_map.draw_box(x.subvector(0, 1), "red");
        fig_map.show(); // argument is robot size
    }


    for(int i=100; i < 200; i++)
    {
        
        // draw robot at actual_x2(t=dt*i)
        IntervalVector vehicle2_i(3);
        vehicle2_i[0] = actual_x2(dt*i)[0];
        vehicle2_i[1] = actual_x2(dt*i)[1];
        vehicle2_i[2] = Interval(-M_PI/2);
        Vector vehicle2 = vehicle2_i.mid();
        fig_map.draw_vehicle(vehicle2, 0.5);
        // fig_map.draw_vehicle(actual_x2(dt*i).subvector(0, 1), 0.5);

        // Generating observations for actual_x2(t=dt*i) of landmarks
        // Display range-and-bearing measurements
        // cout << "actual_x2(dt*i).subvector(0, 2): " << actual_x2(dt*i).subvector(0, 2) << endl;
        // cout << "vehicle2: " << vehicle2 << endl;

        vector<IntervalVector> l1_obs; // Define the 2d range-and-bearing measurements
        vector<IntervalVector> l1_map; // define map of the actual_x2(t=dt*i)

        for(auto& l1_0: l_map)
        {
            Vector l1_d = l1_0.mid() - vehicle2.subvector(0, 1);
            double L1_D = sqrt(l1_d[0] * l1_d[0] + l1_d[1] * l1_d[1]); // landmark to robot distance
            Interval l1_r = L1_D + Interval(-0.1, 0.1); // range-to-landmark observed
            // this l1_phi is only for draw_pie
            double l1_phi1 = atan2(l1_d[1], l1_d[0]); // landmark to robot angle(without robot's heading)
            Interval l1_a1 = l1_phi1 + Interval(-0.04, 0.04); // angle-to-landmark observed
            // this l1_phi is for contraction
            double l1_phi2 = atan2(l1_d[1], l1_d[0]) - vehicle2[2]; // landmark to robot angle(with robot's heading)
            Interval l1_a2 = l1_phi2 + Interval(-0.04, 0.04); // angle-to-landmark observed

            if(L1_D < r_obs)
            {
                l1_obs.push_back({l1_r, l1_a2});
                l1_map.push_back(l1_0);
                fig_map.draw_box(l1_0, "blue[red]");
                // fig_map.draw_pie(vehicle2[0], vehicle2[1], l1_r, l1_a1);
                // fig_map.draw_pie(vehicle2[0], vehicle2[1], (Interval(0.01) | l1_r), l1_a1, "lightGray");
            }
            fig_map.axis_limits(fig_map.view_box(), true, 0.1);        
        }
        fig_map.show();

        // Building contractor
        IntervalVector x(3);
        x[2] = vehicle2[2];
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
        cout << "x = " << x << endl;

        fig_map.draw_box(x.subvector(0, 1), "red");
        fig_map.show(); // argument is robot size
    }

    for(int i=200; i < 300; i++)
    {
        
        // draw robot at actual_x3(t=dt*i)
        IntervalVector vehicle3_i(3);
        vehicle3_i[0] = actual_x3(dt*i)[0];
        vehicle3_i[1] = actual_x3(dt*i)[1];
        vehicle3_i[2] = Interval(-M_PI);
        Vector vehicle3 = vehicle3_i.mid();
        fig_map.draw_vehicle(vehicle3, 0.5);
        // fig_map.draw_vehicle(actual_x3(dt*i).subvector(0, 1), 0.5);

        // Generating observations for actual_x2(t=dt*i) of landmarks
        // Display range-and-bearing measurements
        // cout << "actual_x3(dt*i).subvector(0, 2): " << actual_x3(dt*i).subvector(0, 2) << endl;
        // cout << "vehicle3: " << vehicle3 << endl;

        vector<IntervalVector> l1_obs; // Define the 2d range-and-bearing measurements
        vector<IntervalVector> l1_map; // define map of the actual_x2(t=dt*i)

        for(auto& l1_0: l_map)
        {
            Vector l1_d = l1_0.mid() - vehicle3.subvector(0, 1);
            double L1_D = sqrt(l1_d[0] * l1_d[0] + l1_d[1] * l1_d[1]); // landmark to robot distance
            Interval l1_r = L1_D + Interval(-0.1, 0.1); // range-to-landmark observed
            // this l1_phi is only for draw_pie
            double l1_phi1 = atan2(l1_d[1], l1_d[0]); // landmark to robot angle(without robot's heading)
            Interval l1_a1 = l1_phi1 + Interval(-0.04, 0.04); // angle-to-landmark observed
            // this l1_phi is for contraction
            double l1_phi2 = atan2(l1_d[1], l1_d[0]) - vehicle3[2]; // landmark to robot angle(with robot's heading)
            Interval l1_a2 = l1_phi2 + Interval(-0.04, 0.04); // angle-to-landmark observed

            if(L1_D < r_obs)
            {
                l1_obs.push_back({l1_r, l1_a2});
                l1_map.push_back(l1_0);
                fig_map.draw_box(l1_0, "blue[green]");
                // fig_map.draw_pie(vehicle3[0], vehicle3[1], l1_r, l1_a1);
                // fig_map.draw_pie(vehicle3[0], vehicle3[1], (Interval(0.01) | l1_r), l1_a1, "lightGray");
            }
            fig_map.axis_limits(fig_map.view_box(), true, 0.1);        
        }
        fig_map.show();

        // Building contractor
        IntervalVector x(3);
        x[2] = vehicle3[2];
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
        cout << "x = " << x << endl;

        fig_map.draw_box(x.subvector(0, 1), "red");
        fig_map.show(); // argument is robot size
    }

    for(int i=300; i < 400; i++)
    {
        
        // draw robot at actual_x4(t=dt*i)
        IntervalVector vehicle4_i(3);
        vehicle4_i[0] = actual_x4(dt*i)[0];
        vehicle4_i[1] = actual_x4(dt*i)[1];
        vehicle4_i[2] = Interval(M_PI/2);
        Vector vehicle4 = vehicle4_i.mid();
        fig_map.draw_vehicle(vehicle4, 0.5);
        // fig_map.draw_vehicle(actual_x4(dt*i).subvector(0, 1), 0.5);

        // Generating observations for actual_x2(t=dt*i) of landmarks
        // Display range-and-bearing measurements
        // cout << "actual_x4(dt*i).subvector(0, 2): " << actual_x4(dt*i).subvector(0, 2) << endl;
        // cout << "vehicle4: " << vehicle4 << endl;

        vector<IntervalVector> l1_obs; // Define the 2d range-and-bearing measurements
        vector<IntervalVector> l1_map; // define map of the actual_x2(t=dt*i)

        for(auto& l1_0: l_map)
        {
            Vector l1_d = l1_0.mid() - vehicle4.subvector(0, 1);
            double L1_D = sqrt(l1_d[0] * l1_d[0] + l1_d[1] * l1_d[1]); // landmark to robot distance
            Interval l1_r = L1_D + Interval(-0.1, 0.1); // range-to-landmark observed
            // this l1_phi is only for draw_pie
            double l1_phi1 = atan2(l1_d[1], l1_d[0]); // landmark to robot angle(without robot's heading)
            Interval l1_a1 = l1_phi1 + Interval(-0.04, 0.04); // angle-to-landmark observed
            // this l1_phi is for contraction
            double l1_phi2 = atan2(l1_d[1], l1_d[0]) - vehicle4[2]; // landmark to robot angle(with robot's heading)
            Interval l1_a2 = l1_phi2 + Interval(-0.04, 0.04); // angle-to-landmark observed

            if(L1_D < r_obs)
            {
                l1_obs.push_back({l1_r, l1_a2});
                l1_map.push_back(l1_0);
                fig_map.draw_box(l1_0, "blue[yellow]");
                // fig_map.draw_pie(vehicle4[0], vehicle4[1], l1_r, l1_a1);
                // fig_map.draw_pie(vehicle4[0], vehicle4[1], (Interval(0.01) | l1_r), l1_a1, "lightGray");
            }
            fig_map.axis_limits(fig_map.view_box(), true, 0.1);        
        }
        fig_map.show();

        // Building contractor
        IntervalVector x(3);
        x[2] = vehicle4[2];
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
        cout << "x = " << x << endl;

        fig_map.draw_box(x.subvector(0, 1), "red");
        fig_map.show(); // argument is robot size
    }

    vibes::endDrawing();

}

