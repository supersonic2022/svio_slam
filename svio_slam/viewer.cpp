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

	for (auto kf_it : kfs)
	{
		svo::FramePtr kf = kf_it;
		Sophus::SE3d Twc = kf_it->T_f_w_;

		glPushMatrix();

		//dummy way to convert
		float mat[16];
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				mat[i * 4 + j] = Twc.matrix().transpose()(i,j);
		glMultMatrixf(mat);

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
	}

}
