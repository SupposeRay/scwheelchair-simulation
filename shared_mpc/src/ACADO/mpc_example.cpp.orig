#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

using namespace std;

USING_NAMESPACE_ACADO

int main( )
{
    // States
    DifferentialState X_x, X_y, X_theta;
    Control U_v, U_w;
    DifferentialEquation f;

    // Model (differential equation)
    f << dot(X_x) == U_v * cos(X_theta);
    f << dot(X_y) == U_v * sin(X_theta);
    f << dot(X_theta) == U_w;

    // Least square function
    Function h;
    h << X_x;
    h << X_y;
    h << X_theta;

    // LSQ matrix
    DMatrix Q(3,3);
    Q.setIdentity();

    // Initial conditions
	DVector X0(3);
	X0(0) = 1.0;
	X0(1) = 1.0;
	X0(2) = 0.0;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    // reference vector
    DVector r(3);
    r.setAll( 0.8 );
	// Reference Trajectory
    const double t_start = 0.0;
    const double t_end   = 1.0;	
    Grid timeGrid( t_start, t_end, 11 );
    VariablesGrid   x_ref( 3, timeGrid );
    VariablesGrid   u_ref( 2, timeGrid );    
    double value_x = 0; 
    double value_y = 0; 
    double value_theta = 0; 
    for (int i = 1; i < 10; i++)
    {
        x_ref(0,i) = value_x; 
        x_ref(1,i) = value_y; 
        x_ref(2,i) = value_theta; 
        u_ref(0,i) = 1.0;
        u_ref(1,i) = 0.0;
        value_x += 0.3;
        value_y -= 0.3;
    }
	PeriodicReferenceTrajectory reference_trajectory(x_ref);
	// Optimal Control Problem
    OCP ocp( t_start, t_end, 20 );
    ocp.minimizeLSQ( Q, h, r);
	ocp.subjectTo( f );
    ocp.subjectTo( -M_PI <= X_theta <= M_PI );
    ocp.subjectTo( -2.0 <= U_v <= 2.0 );
    ocp.subjectTo( -2.0 <= U_w <= 2.0 );

    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
	OutputFcn identity;
	DynamicSystem dynamicSystem( f,identity );

	Process process( dynamicSystem,INT_RK45 );

    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
	RealTimeAlgorithm alg( ocp, 0.2 );
	alg.set( MAX_NUM_ITERATIONS, 2 );
	


	Controller controller( alg, reference_trajectory );


    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
	SimulationEnvironment sim( 0.0, 3.0, process,controller );

	DVector x0(4);
	x0(0) = 0.01;
	x0(1) = 0.0;
	x0(2) = 0.0;
	x0(3) = 0.0;

	if (sim.init( x0 ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	if (sim.run( ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

    // ...AND PLOT THE RESULTS
    // ----------------------------------------------------------
	VariablesGrid sampledProcessOutput;
	sim.getSampledProcessOutput( sampledProcessOutput );

	VariablesGrid feedbackControl;
	sim.getFeedbackControl( feedbackControl );

	GnuplotWindow window;
	window.addSubplot( sampledProcessOutput(0), "Body Position [m]" );
	window.addSubplot( sampledProcessOutput(1), "Wheel Position [m]" );
	window.addSubplot( sampledProcessOutput(2), "Body Velocity [m/s]" );
	window.addSubplot( sampledProcessOutput(3), "Wheel Velocity [m/s]" );
	window.addSubplot( feedbackControl(1),      "Damping Force [N]" );
	window.addSubplot( feedbackControl(0),      "Road Excitation [m]" );
	window.plot( );

    return EXIT_SUCCESS;
}



