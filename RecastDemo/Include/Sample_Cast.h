// Copyright (c) 2014 Germain Le Chapelain

#ifndef RECASTSAMPLECAST_H
#define RECASTSAMPLECAST_H

#include "Sample.h"

class Sample_Cast : public Sample
{
public:
	const int MAX_SHAPES = 512;

	struct Point
	{
		Point() {}
		Point( float _x, float _y, float _z ) : x( _x ), y( _y ), z( _z )  {}
		Point( float const * const p )
		{
			for( int i = 0; 3 != i; ++i )
			{
				m_axises[i] = p[i];
			}
		}

		template <typename T>
		static T SQR( T const &a )
		{
			return a * a;
		}

		Point Rotate( float a ) const;

		float operator*( Point const &a ) const
		{
			float return_value = .0f;

			for( int i = 0; 3 != i; ++i )
				return_value += m_axises[i] * a.m_axises[i];

			return return_value;
		}

		Point operator+( Point const &a ) const
		{
			Point return_value( *this );

			for( int i = 0; 3 != i; ++i )
				return_value.m_axises[i] += a.m_axises[i];

			return return_value;
		}

		Point operator-( Point const &a ) const
		{
			Point return_value( *this );

			for( int i = 0; 3 != i; ++i )
				return_value.m_axises[i] -= a.m_axises[i];

			return return_value;
		}

		union
		{
			float m_axises[3];
			struct
			{
				float x, y, z;
			};
		};
	};

	template <typename T, int Capacity>
	struct Pool
	{
		Pool()
		{
			Init();
		}

		void Init()
		{
			m_nextFree = (eIndex)0;
			for( int i = 0; (Capacity - 1) != i; ++i )
			{
				m_elements[i].m_next = (eIndex)( i + 1 );
			}
			m_elements[ Capacity - 1 ].m_next = INVALID;
		}

		typedef enum eIndex { INVALID = -1, MAX = Capacity, } Index;
		struct Node
		{
			T m_element;
			Index m_next;
		};

		Node m_elements[Capacity];

		eIndex m_nextFree;
	};

	bool m_hitBuffer;
	float m_hits[2][1080][2];
	
#if !defined( POOLED_CONSTRAINTS )
	Constraint m_constraints[512];

	int m_numConstraints;
#else
	;
	typedef Pool<Constraint, 12> Constraints;
	static Constraints m_constraints;

#endif
#if !defined( POOLED_CONSTRAINTS )
	bool Constrains( Point const & ) const;
#endif

	struct Shape
	{
#if defined( POOLED_CONSTRAINTS )
		bool Constrains( Point const & ) const;
	}
#endif
	
	Shape()
			: m_radius( .0f )
#if defined( POOLED_CONSTRAINTS )
			, m_constraints( Constraints::INVALID )
#endif
		{}

	float m_center[3];

		float m_radius;

#if defined( POOLED_CONSTRAINTS )
		Constraints::eIndex m_constraints;
#endif
	} m_shapes[512];
	
	int m_numShapes;

	int m_openShapes[128];

	int m_numOpenShapes;

	bool m_showAngles;

	void aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaapushShape( float const p[] );
	
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleRenderOverlay( double* proj, double* model, int* view );
	virtual void handleSettings();
	virtual void handleStep();
	virtual void handleRender();

	Sample_Cast();
	void Init();

	protected:
		template < typename Fun >
		void Cast( Fun & );
};

#endif // RECASTSAMPLECAST_H
