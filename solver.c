#define IX(i,j,k) (i*(N+2)*(N+2)+ j*(N+2) +k)
#define SWAP(x0,x) {float * tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) { for ( k=1;k<=N; k++){
#define END_FOR }}}

#include <stdio.h>
void add_source ( int N, float * x, float * s, float dt )
{
		int i, size= (N+2)*(N+2)*(N+2);
		for ( i=0 ; i<size ; i++ ) x[i] += dt*s[i];
}

void set_bnd ( int N, int b, float * x )
{
		int i,j;

		for ( i=1 ; i<=N ; i++ ) {
				for(j=1;j<=N;j++){
						x[IX(0,i,j)] = b==1 ? -x[IX(1,i,j)] : x[IX(1,i,j)];
						x[IX(N+1,i,j)] = b==1 ? -x[IX(N,i,j)] : x[IX(N,i,j)];
						x[IX(i,0,j)] = b==2 ? -x[IX(i,1,j)] : x[IX(i,1,j)];
						x[IX(i,N+1,j)] = b==2 ? -x[IX(i,N,j)] : x[IX(i,N,j)];
						x[IX(i,j,0)] = b==3 ? -x[IX(i,j,1)] : x[IX(i,j,1)];
						x[IX(i,j,N+1)] = b==3 ? -x[IX(i,j,N)] : x[IX(i,j,N)];
				}
		}
		x[IX(0  ,0  ,0)] = 0.5f*(x[IX(0,0,1)]+x[IX(0,1,0)]);
		x[IX(0  ,0,N+1)] = 0.5f*(x[IX(0,1,N+1)]+x[IX(0,1,N)]);
		x[IX(N+1  ,0,0)] = 0.5f*(x[IX(N+1,0,1)]+x[IX(N+1, 1  ,0)]);
		x[IX(0  ,N+1,0)] = 0.5f*(x[IX(0,N+1,1)]+x[IX(1,N+1,0)]);
		x[IX(N+1,N+1 ,0)] = 0.5f*(x[IX(N+1,N+1,1 )]+x[IX(N+1,N,0)]);
		x[IX(N+1,0 ,N+1)] = 0.5f*(x[IX(N+1,1,N+1 )]+x[IX(N+1,0, N)]);
		x[IX(0,N+1 ,N+1)] = 0.5f*(x[IX(0,N+1,N)]+x[IX(1, N+1,N+1)]);
		x[IX(N+1,N+1, N+1)] = 0.5f*(x[IX(N,N+1,N+1)]+x[IX(N+1,N,N+1)]);
}


void lin_solve ( int N, int b, float * x, float * x0, float a, float c )
{
		int i, j, k, iter;

		for ( iter=0 ; iter<20 ; iter++ ) {
				FOR_EACH_CELL
						x[IX(i,j,k)] = (x0[IX(i,j,k)] + a*(x[IX(i-1,j,k)]+x[IX(i+1,j,k)]+x[IX(i,j-1,k)]+x[IX(i,j+1,k)] + x[IX(i,j,k+1)] + x[IX(i,j,k-1)]))/c;
				END_FOR
						set_bnd ( N, b, x );
		}
}

void diffuse ( int N, int b, float * x, float * x0, float diff, float dt )
{
		float a=dt*diff*N*N*N;
		lin_solve ( N, b, x, x0, a, 1+6*a );
}

void advect ( int N, int b, float * d, float * d0, float * u, float * v, float* w, float dt )
{
		int i, j, i0, j0, i1, j1;
		int k, k0, k1;
		float x, y, s0, t0, s1, t1, dt0;
		float z, u0, u1;

		dt0 = dt*N;
		printf("YES6\n");
		FOR_EACH_CELL
		//printf("%d %d %d %d %f\n",i,j,k, IX(i,j,k), u[IX(i,j,k)]);
				x = i-dt0*u[IX(i,j,k)]; y = j-dt0*v[IX(i,j,k)]; z = k-dt0*w[IX(i,j,k)]; 
		//printf("%f %f %f \n",x,y,z);
		if (x<0.5f) x=0.5f; if (x>N+0.5f) x=N+0.5f; i0=(int)x; i1=i0+1;
		if (y<0.5f) y=0.5f; if (y>N+0.5f) y=N+0.5f; j0=(int)y; j1=j0+1;
		if (z<0.5f) z=0.5f; if (z>N+0.5f) z=N+0.5f; k0=(int)z; k1=k0+1;
		s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1;
		u1 = z-k0; u0 = 1-u1;
		d[IX(i,j,k)] = u0*(s0*(t0*d0[IX(i0,j0,k0)]+t1*d0[IX(i0,j1,k0)])+
						s1*(t0*d0[IX(i1,j0, k0)]+t1*d0[IX(i1,j1, k0)]))+
				u1*(s0*(t0*d0[IX(i0,j0, k1)]+t1*d0[IX(i0,j1, k1)])+
								s1*(t0*d0[IX(i1,j0,k1)]+t1*d0[IX(i1,j1,k1)]));
		END_FOR

		printf("YES7\n");
				set_bnd ( N, b, d );
}

void project ( int N, float * u, float * v, float *w,float * p, float * div )
{
		int i, j,k;
		float _oneThird = 1/3.0f;
		FOR_EACH_CELL
				div[IX(i,j,k)] = -1*_oneThird*(u[IX(i+1,j,k)]-u[IX(i-1,j,k)]+v[IX(i,j+1,k)]-v[IX(i,j-1,k)] +w[IX(i,j,k+1)] -w[IX(i,j,k-1)])/N;
		p[IX(i,j,k)] = 0;
		END_FOR	

				set_bnd ( N, 0, div ); set_bnd ( N, 0, p );

		lin_solve ( N, 0, p, div, 1, 6 );

		printf("YES2\n");
		FOR_EACH_CELL
				u[IX(i,j,k)] -= _oneThird*N*(p[IX(i+1,j,k)]-p[IX(i-1,j,k)]);
		v[IX(i,j,k)] -= _oneThird*N*(p[IX(i,j+1,k)]-p[IX(i,j-1,k)]);
		w[IX(i,j,k)] -= _oneThird*N*(p[IX(i,j,k+1)]-p[IX(i,j,k-1)]);
		END_FOR
		printf("YES2.5\n");
				set_bnd ( N, 1, u ); set_bnd ( N, 2, v );
		printf("YES3\n");
}

void dens_step ( int N, float * x, float * x0, float * u, float * v, float * w, float diff, float dt )
{
		int i,j,k;
		add_source ( N, x, x0, dt );
		//	FOR_EACH_CELL
		//	if(x0[IX(i,j,k)] > 1.0)
		//		printf("%d,%d:%f\n",i,j,x[IX(i,j,k)]);
		//	END_FOR
		SWAP ( x0, x ); diffuse ( N, 0, x, x0, diff, dt );
		SWAP ( x0, x ); advect ( N, 0, x, x0, u, v, w,dt );
}

void vel_step ( int N, float * u, float * v, float *w, float * u0, float * v0, float * w0, float visc, float dt )
{
		int i, j,k;
		add_source ( N, u, u0, dt ); add_source ( N, v, v0, dt ); add_source(N, w, w0, dt);
		SWAP ( u0, u ); diffuse ( N, 1, u, u0, visc, dt );
		SWAP ( v0, v ); diffuse ( N, 2, v, v0, visc, dt );
		SWAP ( w0, w ); diffuse ( N, 3, w, w0, visc, dt );
			FOR_EACH_CELL
			if(u[IX(i,j,k)] > 0.0)
				printf("%d,%d,%d:%f-",i,j,k,u[IX(i,j,k)]);
			END_FOR
				printf("\n");
		//printf("u0:%f\n",u0[IX(1,1,1)]);
		project ( N, u, v, w, u0, v0);
		//printf("YES4\n");
		//printf("u:%f\n",u0[IX(1,1,1)]);
		SWAP ( u0, u ); SWAP ( v0, v ); SWAP ( w0, w );
		advect ( N, 1, u, u0, u0, v0, w0,dt ); advect ( N, 2, v, v0, u0, v0,w0, dt ); advect(N,3,w,w0,u0,v0,w0,dt);
		project ( N, u, v, w, u0, v0);
}

