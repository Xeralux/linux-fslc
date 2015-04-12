ifeq ($(CONFIG_VIDEO_MXC_IPU_CAMERA),y)
	obj-$(CONFIG_VIDEO_MXC_CAPTURE) += mxc_v4l2_capture.o
	obj-$(CONFIG_MXC_IPU_PRP_VF_SDC) += ipu_prp_vf_sdc.o ipu_prp_vf_sdc_bg.o
	obj-$(CONFIG_MXC_IPU_DEVICE_QUEUE_SDC) += ipu_fg_overlay_sdc.o ipu_bg_overlay_sdc.o
	obj-$(CONFIG_MXC_IPU_PRP_ENC) += ipu_prp_enc.o ipu_still.o
	obj-$(CONFIG_MXC_IPU_CSI_ENC) += ipu_csi_enc.o ipu_still.o
endif

camera_sensor_clock-objs := sensor_clock.o
obj-$(CONFIG_MXC_CAMERA_SENSOR_CLK) += camera_sensor_clock.o

ov5640_camera_int-objs := ov5640.o
obj-$(CONFIG_MXC_CAMERA_OV5640) += ov5640_camera_int.o

ov5642_camera-objs := ov5642.o
obj-$(CONFIG_MXC_CAMERA_OV5642) += ov5642_camera.o

ov5640_camera_mipi-objs := ov5640_mipi.o
obj-$(CONFIG_MXC_CAMERA_OV5640_MIPI) += ov5640_camera_mipi.o

obj-$(CONFIG_MXC_CAMERA_SUBDEV_PIPELINE) += mxc_subdev_pipeline.o
obj-$(CONFIG_MXC_CAMERA_SUBDEV_MIPI) += mxc_subdev_mipi.o
obj-$(CONFIG_MXC_CAMERA_SUBDEV_PARALLEL) += mxc_subdev_parallel.o

adv7180_tvin-objs := adv7180.o
obj-$(CONFIG_MXC_TVIN_ADV7180) += adv7180_tvin.o

obj-$(CONFIG_VIDEO_V4L2_MXC_INT_DEVICE) += v4l2-int-device.o

tda1997x-video-objs := tda1997x.o
obj-$(CONFIG_MXC_TVIN_TDA1997X) += tda1997x-video.o
obj-$(CONFIG_SSMN_AP0100_M034) += ap0100_m034.o
