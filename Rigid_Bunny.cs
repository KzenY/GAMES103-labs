using UnityEngine;
using System.Collections;
using System;

public class Rigid_Bunny : MonoBehaviour
{
	bool launched = false;
	float dt = 0.015f;
	Vector3 v = new Vector3(0, 0, 0);   // velocity
	Vector3 w = new Vector3(0, 0, 0);   // angular velocity

	float mass;                                 // mass
	Matrix4x4 I_ref;                            // reference inertia

	float linear_decay = 0.999f;                // for velocity decay
	float angular_decay = 0.98f;
	float restitution = 0.5f;                 // for collision
	float friction = 0.2f;

	Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);


	// Use this for initialization
	void Start()
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m = 1;
		mass = 0;
		for (int i = 0; i < vertices.Length; i++)
		{
			mass += m;
			float diag = m * vertices[i].sqrMagnitude;
			I_ref[0, 0] += diag;
			I_ref[1, 1] += diag;
			I_ref[2, 2] += diag;
			I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
			I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
			I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
			I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
			I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
			I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
			I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
			I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
			I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
		}
		I_ref[3, 3] = 1;
	}

	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A[0, 0] = 0;
		A[0, 1] = -a[2];
		A[0, 2] = a[1];
		A[1, 0] = a[2];
		A[1, 1] = 0;
		A[1, 2] = -a[0];
		A[2, 0] = -a[1];
		A[2, 1] = a[0];
		A[2, 2] = 0;
		A[3, 3] = 1;
		return A;
	}

	Matrix4x4 Matrix_subtract(Matrix4x4 a,Matrix4x4 b)
	{
		for(int i = 0; i < 4; i++)
		{
			for(int j = 0; j < 4; j++)
			{
				a[i, j] -= b[i, j];
			}
		}
		return a;
	}

	Matrix4x4 Matrix_multiply_float(Matrix4x4 a,float b)
	{
		for(int i = 0; i < 4; i++)
		{
			for(int j = 0; j < 4; j++)
			{
				a[i, j] *= b;
			}
		}
		return a;
	}

	Quaternion Add(Quaternion a,Quaternion b)
	{
		a.x += b.x;
		a.y += b.y;
		a.z += b.z;
		a.w += b.w;
		return a;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		//物体顶点
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;
		//物体位置（全局坐标质心
		Vector3 pos = transform.position;
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		//碰撞点
		Vector3 P_colli = new Vector3(0, 0, 0);
		//碰撞数
		int ColliNum = 0;

		for (int i = 0; i < vertices.Length; i++)
		{
			Vector3 r_i = vertices[i];
			//旋转后顶点
			Vector3 R_i = R.MultiplyVector(r_i);
			//旋转后顶点+物体位置=顶点在世界空间中位置
			Vector3 x_i = pos + R_i;
			float d = Vector3.Dot(x_i - P, N);
			if (d < 0.0f)
			{
				//角速度叉乘矢径，计算受碰撞影响的顶点速度
				Vector3 v_i = v + Vector3.Cross(w, R_i);
				//碰撞点速度法向分量
				float v_N_size = Vector3.Dot(v_i, N);
				if (v_N_size < 0.0f)
				{
					P_colli += r_i;
					ColliNum++;
				}
			}

		}
		if (ColliNum == 0) return;
		//平均碰撞点
		Vector3 r_colli = P_colli / ColliNum;
		Vector3 Rr_colli = R.MultiplyVector(r_colli);
		Vector3 v_colli = v + Vector3.Cross(w, Rr_colli);
		
		Vector3 v_N = Vector3.Dot(v_colli,N) * N;
		Vector3 v_T = v_colli - v_N;
		//碰撞后速度
		Vector3 v_N_new = -1.0f*restitution * v_N;
		float a = Math.Max(1.0f - friction * (1.0f + restitution) * v_N.magnitude / v_T.magnitude, 0.0f);
        Vector3 v_T_new = a*v_T;
		Vector3 v_new = v_T_new + v_N_new;
        //惯性张量
        Matrix4x4 I_rot = R * I_ref * Matrix4x4.Transpose(R);
        //计算碰撞冲量
        Matrix4x4 Rri_ = Get_Cross_Matrix(Rr_colli);
		Matrix4x4 K = Matrix_subtract(Matrix_multiply_float(Matrix4x4.identity, 1.0f / mass), Rri_ * I_rot.inverse * Rri_);
		Vector3 j = K.inverse.MultiplyVector(v_new - v_colli);
		//更新速度和角速度
		v += 1.0f / mass * j;
		w += I_rot.inverse.MultiplyVector(Vector3.Cross(Rr_colli, j));
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}
		if (launched) { 
		// Part I: Update velocities
		v += gravity * dt;
		v *= linear_decay;
		w *= angular_decay;

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x  = transform.position;
		x += dt * v;
		//Update angular status
		Quaternion q = transform.rotation;
		Vector3 dw = 0.5f * dt * w;
		Quaternion qw = new Quaternion(dw.x, dw.y, dw.z, 0.0f);
		q = Add(q, qw * q);
		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	    }
    }
}
