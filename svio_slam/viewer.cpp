#include "viewer.h"
#include "global.h"
#include "frame.h"
#include "point.h"
#include <pangolin/pangolin.h>

Viewer::Viewer(vk::AbstractCamera* cam_, svo::FrameHandlerMono* vo_):
	cam(cam_), vo(vo_)
{
	width = 1024;
	height = 768;
	double fps = 30;
	mT = 1e3 / fps;

}

void Viewer::run()
{
	pangolin::CreateWindowAndBind("svio", width, height);
	glEnable(GL_DEPTH_TEST);

	pangolin::OpenGlRenderState s_cam(
		pangolin::ProjectionMatrix(width, height, 500, 500, width / 2, height / 2, 0.1, 1000),
		pangolin::ModelViewLookAt(0, -0.7, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
	);

	pangolin::View& d_cam = pangolin::CreateDisplay()
		.SetBounds(0.0, 1.0, 0.0, 1.0, -width / height)
		.SetHandler(new pangolin::Handler3D(s_cam));

	pangolin::OpenGlMatrix Twc;
	Twc.SetIdentity();
	
	while (1)
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		d_cam.Activate(s_cam);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

		drawKFs();

		pangolin::FinishFrame();

		cv::waitKey(mT);
	}
		
}

void Viewer::drawKFs()
{
	const float w = 0.05;
	const float h = w * 0.75;
	const float z = w * 0.6;

	auto& kfs = (vo->map()).keyframes_;
	if (!kfs.size())
		return;

	//std::cout << "kf size = " << kfs.size() << std::endl;

	bool fisrt_kf = true;
	GLfloat cur_mat[16];
	GLfloat ow1[3] = {0 ,0 ,0}, ow2[3];

	for (auto kf_it : kfs)
	{
		svo::FramePtr kf = kf_it;
		Sophus::SE3d Twc = kf_it->T_f_w_;

		glPushMatrix();

		//dummy way to convert
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				cur_mat[i * 4 + j] = Twc.matrix().transpose()(i, j);
		ow2[0] = cur_mat[12];
		ow2[1] = cur_mat[13];
		ow2[2] = cur_mat[14];


		glMultMatrixf(cur_mat);

		glLineWidth(1.0f);
		glColor3f(0.0f, 0.0f, 1.0f);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(w, h, z);
		glVertex3f(0, 0, 0);
		glVertex3f(w, -h, z);
		glVertex3f(0, 0, 0);
		glVertex3f(-w, -h, z);
		glVertex3f(0, 0, 0);
		glVertex3f(-w, h, z);

		glVertex3f(w, h, z);
		glVertex3f(w, -h, z);

		glVertex3f(-w, h, z);
		glVertex3f(-w, -h, z);

		glVertex3f(-w, h, z);
		glVertex3f(w, h, z);

		glVertex3f(-w, -h, z);
		glVertex3f(w, -h, z);
		glEnd();	
		glPopMatrix();

		if (fisrt_kf) 
			fisrt_kf = false;
		else
		{		
			glLineWidth(1.0f);
			glColor3f(0.0f, 1.0f, 0.0f);
			glBegin(GL_LINES);
			glVertex3f(ow1[0], ow1[1], ow1[2]);
			glVertex3f(ow2[0], ow2[1], ow2[2]);
			glEnd();
		}

		ow1[0] = ow2[0];
		ow1[1] = ow2[1];
		ow1[2] = ow2[2];
	}
}
