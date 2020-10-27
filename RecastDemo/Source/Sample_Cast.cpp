// Copyright (c) 2014 Germain Le Chapelain

#include <stdio.h>
#include "SDL_opengl.h"

#include "imgui.h"
#include "InputGeom.h"

#define _USE_MATH_DEFINES

#include <math.h>
#include "NavMeshTesterTool.h"

#include "Sample_Cast.h"

Sample_Cast::Point operator*( float a, Sample_Cast::Point const &p )
{
	Sample_Cast::Point return_value( p );

	for( int i = 0; 3 != i; ++i )
		return_value.m_axises[i] *= a;
		
	return return_value;
}

void Sample_Cast::Init()
{
	m_numShapes = 0;
	m_numOpenShapes = 0;
	m_numConstraints = 0;
}

Sample_Cast::Sample_Cast()
: m_hitBuffer( false )
, m_showAngles( false )
{
	Init();
	setTool(new NavMeshTesterTool);
}

void Sample_Cast::handleRenderOverlay(double* proj, double* model, int* view)
{
	//GLdouble x, y, z;

	//for( int i_shape = 0; m_numShapes != i_shape; ++i_shape )
	//{
	//	char lbl_shape[230];
	//	Shape const &shape = m_shapes[i_shape];
	//	if( gluProject( shape.m_center.x, shape.m_center.y, shape.m_center.z,
	//					model, proj, view, &x, &y, &z ) )
	//	{
	//		sprintf( lbl_shape, "Shape n. %d", i_shape );
	//		imguiDrawText((int)x, (int)(y - 25), IMGUI_ALIGN_CENTER, lbl_shape, imguiRGBA(0, 0, 0, 220));
	//	}
	//
	//}

	Sample::handleRenderOverlay( proj, model, view );
}

float shape_radius;
static int   const tesselation = 6;
static float const		  step = 2.f;

class RenderCast
{
public:
	void ReCast( bool hit, bool constrained, 
			   const float *center,
			   const float *p, const float *p_prev, const float *p_next,
			   int a, float angle, float abs_f_on_reach ,
			   const float *d, const float *norm ) {}
	void Cast( bool hit, bool constrained, 
			   const float *center,
			   const float *p, const float *p_prev, const float *p_next,
			   int a, float angle, float abs_f_on_reach ,
			   const float *d, const float *norm )
	{
		glBegin( GL_LINE_STRIP );

		if( constrained )
		{
			glColor3f( 1, 0, 0 );
		}
		else if( hit )
		{
			glColor3f( 1, 1, 1 );
		}
		else
		{
			glColor3f( 0, 0, 0 );
		}

		if (hit)
		{

			glVertex3fv( d );
		}
		else if (!constrained)
		{
			glVertex3fv( p_prev );
		}

		glVertex3fv( p );
		glVertex3fv( p_next );

		glEnd();
		if( hit )
		{
			float abs_norm[3]; rcVadd( abs_norm, d, norm );

			glBegin( GL_LINE_STRIP );

			glColor3f(0, 1, 0);

			glVertex3fv( d );
			glVertex3fv( abs_norm );
			
			glEnd();
		}
	}
};

void Sample_Cast::handleRender()
{
/*	for( int i_openShape = 0; m_numOpenShapes != i_openShape; ++i_openShape )
	{
		Point const &center = m_shapes[m_openShapes[i_openShape]].m_center;
		*/
	for( int i_shape = 0; m_numShapes != i_shape; ++i_shape )
	{
		Shape &shape = m_shapes[i_shape];
#if defined( POOLED_CONSTRAINTS )
		for( Constraints::eIndex i_constraint = shape.m_constraints; Constraints::INVALID != i_constraint; i_constraint = m_constraints.m_elements[i_constraint].m_next )
		{
			Constraint const &constraint = m_constraints.m_elements[i_constraint].m_element;
#else
		for( int i = 0; m_numConstraints != i; ++i )
		{
			Constraint const   & constraint = m_constraints[i];
#endif
			Point const constraint_center = Point( shape.m_center ) + Point( constraint.m_distance, .0f, .0f ).Rotate( constraint.m_angle );

			Point const			extremity = Point( 1.f, .0f, .0f ).Rotate( constraint.m_angle + M_PI_2 );
			Point const				start = constraint_center + ( constraint.m_minSet ? constraint.m_min : - 5 ) * extremity;
			Point const				  end = constraint_center + ( constraint.m_maxSet ? constraint.m_max :   5 ) * extremity;

			glLineWidth( 5.f );
			glBegin( GL_LINES );
			if( !constraint.m_minSet )
				glColor3f( 1, 1, 1 );
			else
				glColor3f( 0, 0, 1 );

			glVertex3fv( start.m_axises );
			if( !constraint.m_maxSet )
				glColor3f( 1, 1, 1 );
			else
				glColor3f( 0, 0, 1 );
			glVertex3fv(   end.m_axises );

			glEnd();
			glLineWidth(1.0f);
		}
	}

	// Cast( RenderCast() );

	Sample::handleRender();
}

template < typename Fun >
void Sample_Cast::Cast( Fun & fun )
{
	if( m_numOpenShapes > 0 )
	{
		Shape const & shape = m_shapes[ m_openShapes[ m_numOpenShapes - 1 ] ];

		int	  const	    rad = int( ( shape.m_radius + step ) * tesselation );
		int	  const	old_rad = int(   shape.m_radius		     * tesselation );
		float const  to_rad = 1.f / rad * 2 * M_PI;

		for( int a = 0; rad != a; ++a )
		{
			static int reach = 8;

			float prev_hit[2][2] =
			{
				{
					0,
					shape.m_center[1],
				},
				{
					0,
					shape.m_center[1],
				},
			};

			if( shape.m_radius > 0 )
			{
				for( int i = 0; 2 != i; ++i )
				{
					float const	f_prev_rad = float( a + i ) / rad * old_rad;
					int   const	  prev_rad = int( f_prev_rad );

					float const contrib_next = f_prev_rad - prev_rad;

					for( int j = 0; 2 != j; ++j )
					{
						prev_hit[i][j] = ( 1.f - contrib_next ) * m_hits[!m_hitBuffer][   prev_rad		 % old_rad ][j]
											+	 contrib_next   * m_hits[!m_hitBuffer][ ( prev_rad + 1 ) % old_rad ][j];
					}
				}
			}

			for( int f = 0; 2 * reach != f; ++f )
			{
				float const       f_on_reach = 2.f * f / reach;
				float const     f_1_on_reach = 2.f * (f + 1) / reach;
				float const   abs_f_on_reach = f_on_reach + prev_hit[0][0];
				float const abs_f_on_reach_1 = f_on_reach + prev_hit[1][0];
				float const abs_f_1_on_reach = f_1_on_reach + prev_hit[0][0];

				float const			   angle = to_rad * a;
				float const			 angle_1 = to_rad * ( a + 1 );

				float p_prev[3] =
				{
					abs_f_1_on_reach * cosf( angle ) + shape.m_center[0],
					1.f - powf( f_1_on_reach - 1.f, 2 ) + prev_hit[0][1],
					abs_f_1_on_reach * sinf( angle ) + shape.m_center[2],
				};

				float p[3] = 
				{
					abs_f_on_reach * cosf( angle ) + shape.m_center[0],
					1.f - powf( f_on_reach - 1.f, 2 ) + prev_hit[0][1],
					abs_f_on_reach * sinf( angle ) + shape.m_center[2],
				};

				float p_next[3] = 
				{
					abs_f_on_reach_1 * cosf( angle_1 ) + shape.m_center[0],
					1.f - powf( f_on_reach - 1.f, 2 ) + prev_hit[1][1],
					abs_f_on_reach_1 * sinf( angle_1 ) + shape.m_center[2],
				};

				float hitt = 1.f;

				bool const constrained = Constrains( p );
				float norm[3];
				bool const hit = !constrained && m_geom->raycastMesh( p, p_prev, hitt, norm );
				float d[3];
				rcVsub( d, p_prev, p );
				rcVmad( d, p, d, hitt );

				fun.Cast( hit, constrained, shape.m_center, p, p_prev, p_next, a, angle, abs_f_on_reach, d, norm );

				if( constrained || hit )
				{
					fun.ReCast( hit, constrained, shape.m_center, p, p_prev, p_next, a, angle, abs_f_on_reach, d, norm );
					break;
				}
			}
		}
	}
}

void Sample_Cast::pushShape( float const p[] )
{
	m_openShapes[m_numOpenShapes++] = m_numShapes;
	rcVcopy( m_shapes[m_numShapes++].m_center, p );
}

void Sample_Cast::handleClick(const float* s, const float* p, bool shift)
{
	if( m_numOpenShapes )
	{
		Sample::handleClick(s, p, shift);
	}
	else
	{
	}
}

#if defined( POOLED_CONSTRAINTS )
Sample_Cast::Constraints Sample_Cast::m_constraints;
#endif

#if defined( POOLED_CONSTRAINTS )
bool Sample_Cast::Shape::Constrains( Point const &a ) const
{
	for( Constraints::eIndex i_constraint = m_constraints; Constraints::INVALID != i_constraint; i_constraint = Sample_Cast::m_constraints.m_elements[i_constraint].m_next )
	{
		Constraint const   & constraint = Sample_Cast::m_constraints.m_elements[i_constraint].m_element;
#else
bool Sample_Cast::Constrains( Point const &a ) const
{
	for( int i = 0; m_numConstraints != i; ++i )
	{
		Constraint const   & constraint = m_constraints[i];
#endif
		Point	   const dir_constraint = Point( 1.f, .0f, .0f ).Rotate( constraint.m_angle );
		Point	   const pos_constraint = Point( m_shapes[ m_openShapes[ m_numOpenShapes - 1 ] ].m_center ) + constraint.m_distance * dir_constraint;

		Point	  const a_in_constraint = a - pos_constraint;

		if( a_in_constraint * dir_constraint >= 0 )
		{
			return true;
		}
	}

	return false;
}

class ProbeCast
{
	
	Sample_Cast * m_sample;

public:
	bool m_completelyConstrained;

	ProbeCast( Sample_Cast * main )
		: m_sample( main )
		, m_completelyConstrained( true )
	{}
		
	void Cast( bool hit, bool constrained, 
			   const float *center,
			   const float *p, const float *p_prev, const float *p_next,
			   int a, float angle, float abs_f_on_reach ,
			   const float *d, const float *norm ) {}
	void ReCast( bool hit, bool constrained, 
			   const float *center,
			   const float *p, const float *p_prev, const float *p_next,
			   int a, float angle, float abs_f_on_reach ,
			   const float *d, const float *norm )
	{
		if( !constrained )
		{
			m_completelyConstrained = false;
		}

		if( hit )
		{
			if( norm[1] < .5f )
			{
#if defined( POOLED_CONSTRAINTS )
				Constraints::eIndex current_constraints = shape.m_constraints;
				C
				shape.m_constraints = m_constraints.m_nextFree;
				m_constraints.m_nextFree = m_constraints.m_elements[m_constraints.m_nextFree].m_next;
				m_constraints.m_elements[shape.m_constraints].m_next = current_constraints;

				Constraint & constraint = m_constraints.m_elements[shape.m_constraints].m_element;
#else

				int i = m_sample->m_numConstraints;
				Sample_Cast::Constraint constraint;

#endif

				float center_in_hit[3]; rcVsub( center_in_hit, center, d ); center_in_hit[1] = .0f;
				float flattened_norm[3] = { norm[0], .0f, norm[2] }; rcVnormalize( flattened_norm );
				constraint.m_angle = atan2f( - norm[2], - norm[0] );
				constraint.m_distance = rcVdot( center_in_hit, flattened_norm );
				constraint.m_maxSet = constraint.m_minSet = false;

				Sample_Cast::Shape & shape = m_sample->m_shapes[ m_sample->m_openShapes[ m_sample->m_numOpenShapes - 1 ] ];

				float const x1 = shape.m_center[0] + constraint.m_distance * cosf( constraint.m_angle );
				float const y1 = shape.m_center[2] + constraint.m_distance * sinf( constraint.m_angle );

				float const a1 = cosf( constraint.m_angle + M_PI_2 );
				float const b1 = sinf( constraint.m_angle + M_PI_2 );

				float const x2 = x1 + a1;
				float const y2 = y1 + b1;

				for( int j = 0; j < m_sample->m_numConstraints; ++j )
				{
					if( constraint.m_angle == m_sample->m_constraints[j].m_angle )
					{
						if( constraint.m_distance < m_sample->m_constraints[j].m_distance )
						{
							m_sample->m_constraints[j] = m_sample->m_constraints[--m_sample->m_numConstraints];
							continue;
						}
						else
						{
							return ;
						}
					}
					float const x3 = shape.m_center[0] + m_sample->m_constraints[j].m_distance * cosf( m_sample->m_constraints[j].m_angle );
					float const y3 = shape.m_center[2] + m_sample->m_constraints[j].m_distance * sinf( m_sample->m_constraints[j].m_angle );

					float const a2 = cosf( m_sample->m_constraints[j].m_angle + M_PI_2 );
					float const b2 = sinf( m_sample->m_constraints[j].m_angle + M_PI_2 );

					float const x4 = x3 + a2;
					float const y4 = y3 + b2;
					
					float const dx = ( ( x1 - x2 ) * ( y3 - y4 ) - ( y1 - y2 ) * ( x3 - x4 ) );
					float const dy = ( ( x1 - x2 ) * ( y3 - y4 ) - ( y1 - y2 ) * ( x3 - x4 ) );

					if( .0f == dx || .0f == dy )
					{
						continue;
					}

					float const x  = ( ( x1 * y2 - y1 * x2 ) * ( x3 - x4 ) - ( x1 - x2 ) * ( x3 * y4 - y3 * x4 ) )
											 / dx;

					float const y  = ( ( x1 * y2 - y1 * x2 ) * ( y3 - y4 ) - ( y1 - y2 ) * ( x3 * y4 - y3 * x4 ) )
											 / dy;

					float const p1  = ( x - x1 ) * a1 + ( y - y1 ) * b1;
					float const p2  = ( x - x3 ) * a2 + ( y - y3 ) * b2;

					float diff = m_sample->m_constraints[j].m_angle - constraint.m_angle;
					if (diff > M_PI)
					{
						diff -= 2 * M_PI;
					}
					if (diff < - M_PI)
					{
						diff += 2 * M_PI;
					}

					if( diff > 0 )
					{
						if( !constraint.m_maxSet
							|| p1 < constraint.m_max )
						{
							constraint.m_maxSet = true;
							constraint.m_max = p1;
						}

						if( !m_sample->m_constraints[j].m_minSet
							|| p2 > m_sample->m_constraints[j].m_min)
						{
							m_sample->m_constraints[j].m_minSet = true;
							m_sample->m_constraints[j].m_min = p2;
						}
					}
					else
					{
						if( !constraint.m_minSet
							|| p1 > constraint.m_min )
						{
							constraint.m_minSet = true;
							constraint.m_min = p1;
						}

						if( !m_sample->m_constraints[j].m_maxSet
							|| p2 < m_sample->m_constraints[j].m_max )
						{
							m_sample->m_constraints[j].m_maxSet = true;
							m_sample->m_constraints[j].m_max = p2;
						}
					}

					if(	   m_sample->m_constraints[j].m_minSet
						&& m_sample->m_constraints[j].m_maxSet
						&& m_sample->m_constraints[j].m_max <= m_sample->m_constraints[j].m_min)
					{
						m_sample->m_constraints[j] = m_sample->m_constraints[--m_sample->m_numConstraints];
						--j;
					}
				}

				if(	   !constraint.m_minSet
					|| !constraint.m_maxSet
					|| constraint.m_max > constraint.m_min)
				{
					m_sample->m_constraints[m_sample->m_numConstraints++] = constraint;
				}
			}

		}

		float hit_in_center[3]; rcVsub( hit_in_center, d, center );

		m_sample->m_hits[m_sample->m_hitBuffer][a][0] =
			rcSqrt(
					rcSqr( hit_in_center[0] )
				  + rcSqr( hit_in_center[2] ) );
		m_sample->m_hits[m_sample->m_hitBuffer][a][1] = d[1];
	}

};

void Sample_Cast::handleStep()
{
	Shape		 & shape = m_shapes[ m_openShapes[ m_numOpenShapes - 1 ] ];
	ProbeCast probe_cast ( this );

	// Cast( probe_cast );
	Constraint constraint;
	bool const constraint_added = m_geom->getClosestPoint( shape.m_center,
														   &m_constraints[m_numConstraints], m_numConstraints,
														   constraint ) );
	if( constraint_added )
	{
		int spot_constraint = 0;
		for( ; m_numConstraints != spot_constraint; ++spot_constraint )
		{
			if( RadDist( constraint.m_angle, m_constraints[spot_constraint] ) < 0 )
			{
				break;
				
			}
			
		}

		for( int j = m_numConstraints; spot_constraint != j; --j )
		{
			m_constraints[ j + 1 ] = m_constraints[ j ];
		}

		m_constraints[spot_constraint] = constraint;

		++m_numConstraints;

	}


	m_hitBuffer = !m_hitBuffer;
	if( !probe_cast.m_completelyConstrained )
	{
		shape.m_radius += step;
	}
}

Sample_Cast::Point Sample_Cast::Point::Rotate( float a ) const
{
	float const current = atan2f( this->z, this->x );
	float const	   dist = sqrtf( SQR( this->z ) + SQR( this->x ) );

	float const	  total = current + a;

	return ( dist * Point( cosf( total ), .0f, sinf( total ) ) ) + Point( .0f, this->y, .0f );
}

void Sample_Cast::handleSettings()
{
	// Sample::handleCommonSettings();

	imguiSlider( "Shape Radius", &shape_radius, 0.0f, 50.0f, .01f);
	m_showAngles = imguiCheck( "Show Angle", m_showAngles );
	char text[64];
	snprintf( text, 64, "Constraints  %d", m_numConstraints );
	imguiValue(text);

	Shape & shape = m_shapes[ 0 ];
	snprintf( text, 64, "Heights  %d", int( ( shape.m_radius + 1.f ) * 12 ) );
	imguiValue(text);

	if (imguiButton("Re-Init Shape"))
	{
#if defined( POOLED_CONSTRAINTS )
		m_constraints.Init();
#else
		m_numConstraints = 0;
#endif
		if (m_numOpenShapes)
		{
			m_shapes[ m_openShapes[ m_numOpenShapes - 1 ] ].m_radius = 0;
		}
	}
	if (imguiButton("Clear"))
	{
		m_numOpenShapes = 0;
		m_numShapes = 0;

#if defined( POOLED_CONSTRAINTS )
		m_constraints.Init();
#else
		m_numConstraints = 0;
#endif
	}

}
