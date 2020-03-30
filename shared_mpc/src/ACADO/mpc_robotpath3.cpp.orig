#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include<math.h>
using namespace std;

int main( )
{
  USING_NAMESPACE_ACADO
    // DEFINE THE VARIABLES:
    // ----------------------------------------------------------
    DifferentialState   x    ;  // position in x-direction
    DifferentialState   y    ;  // position in y-direction
    DifferentialState   theta  ;  // robot orientation
    Control             v    ;  // control commands
    Control             w    ;  // control commands

    const double T = 0.25; // horizon step length
    const int N = 10; // prediction horizon steps

    const double t_sim_step = T;    
    const double t_sim_total = 10;

    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    DiscretizedDifferentialEquation f(T);
    f << next( x     )  == x+ v*cos(theta)*T;
    f << next( y     )  == y+ v*sin(theta)*T;
    f << next( theta )  == theta+w*T;

    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
    OutputFcn identity;
    DynamicSystem dynamicSystem( f,identity );
    Process process( dynamicSystem, INT_RK45);

    // DEFINE AN OPTIMAL CONTROL PROBLEM
    // ----------------------------------
    Function h; 
    h << x; 
    h << y;
    h << theta; 
    h << v; 
    h << w;

    DMatrix Q(5,5); // LSQ coefficient DMatrix
    Q.setIdentity();
    Q(0,0) = 0.2; 
    Q(1,1) = 2; 
    Q(2,2) = 0.2; 
    Q(3,3) = 2; 
    Q(4,4) = 2.5;

    // reference 
    DVector ref(5); 
    ref.setAll( 0.0 );

    // define the prediction horizon 
    const double tStart = 0.0;
    const double tEnd   = N*T;

	// Reference Trajectory
    const double t_start = 0.0;
    const double t_end   = 1.0;	
    Grid timeGrid( t_start, t_end, 11 );
    VariablesGrid   x_ref( 3, 11 );
    VariablesGrid   u_ref( 2, 11 );    
    double value_x = 0; 
    double value_y = 0; 
    double value_theta = 0; 
    for (int i = 1; i < 10; i++)
    {
        x_ref(0,i) = value_x; 
        // x_ref(1,i) = value_y; 
        // x_ref(2,i) = value_theta; 
        // u_ref(0,i) = 1.0;
        // u_ref(1,i) = 0.0;
        value_x += 0.3;
        value_y -= 0.3;
    }
	// PeriodicReferenceTrajectory reference_trajectory(x_ref);

    // optimal control problem
    OCP ocp( tStart, tEnd, N );
    ocp.minimizeLSQ( Q, h, ref );
    ocp.subjectTo( f ); 
    ocp.subjectTo( -0.5 <= v <= 0.5 );
    ocp.subjectTo( -0.8 <= w <= 0.8 );

    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
    RealTimeAlgorithm alg( ocp,t_sim_step );

    StaticReferenceTrajectory zeroReference;
    Controller controller( alg, zeroReference );

    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
    SimulationEnvironment sim( 0.0,t_sim_total,process,controller );

    DVector x0(3);
    x0.setZero();
    x0(0) = 2; 
    x0(1) = -2; 
    x0(2) = 3.14;

    sim.init( x0 ); 
    sim.run( );

    // ... AND PLOT THE RESULTS
    // ------------------------
    VariablesGrid diffStates;
    sim.getProcessDifferentialStates( diffStates );

    VariablesGrid feedbackControl;
    sim.getFeedbackControl( feedbackControl );

    diffStates.print();

    GnuplotWindow window;
    window.addSubplot( diffStates(0),   "x [m]" );
    window.addSubplot( diffStates(1),   "y [m]" );
    window.addSubplot( diffStates(2),   "theta [rad]" );
    window.addSubplot( feedbackControl(0), "lin_vel [m/s]" );
    window.addSubplot( feedbackControl(1), "ang_vel [rad/s]" );
    window.plot( );

    return 0;
}