import sys
sys.path.append('/home/mihawk/FoundationPose')


from ultralytics import YOLO
from estimater import *
from datareader import *
import argparse
import json
# model = YOLO("./runs/segment/train8/weights/best.pt")
# results = model("/home/mihawk/yolo/dataset1/images/0001.png", save=False, imgsz=320, conf=0.5)
 
# for result in results:
#     boxes = result.boxes
#     probs = result.probs
#     result.show()
    
# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import socket




if __name__=='__main__':
    parser = argparse.ArgumentParser()
    code_dir = os.path.dirname(os.path.realpath(__file__))
    # parser.add_argument('--mesh_file', type=str, default='/home/mihawk/Cutie/dataset/Carton_Model_V3.obj')
    # parser.add_argument('--mesh_file', type=str, default='/home/mihawk/Cutie/dataset/Bottle_V1_cola2.obj')
    parser.add_argument('--mesh_file', type=str, default='/home/mihawk/Cutie/dataset/Can_Model_V4.obj')
    
    parser.add_argument('--test_scene_dir', type=str, default='/home/mihawk/Cutie/dataset1')
    parser.add_argument('--est_refine_iter', type=int, default=5)
    parser.add_argument('--track_refine_iter', type=int, default=2)
    parser.add_argument('--debug', type=int, default=1)
    parser.add_argument('--debug_dir', type=str, default=f'{code_dir}/debug')
    args = parser.parse_args()
    # video_writer = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (640, 480))
    set_logging_format()
    set_seed(0)

    mesh = trimesh.load(args.mesh_file,force='mesh')
    mesh.vertices *= 2
    # mesh.vertices /=1000
    find_bool = False
    first_bool = True
    debug = args.debug
    debug_dir = args.debug_dir
    os.system(f'rm -rf {debug_dir}/* && mkdir -p {debug_dir}/track_vis {debug_dir}/ob_in_cam')

    to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
    bbox = np.stack([-extents/2, extents/2], axis=0).reshape(2,3)
    
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    host = '127.0.0.1'  
    port = 8888     
    sock.connect((host, port))
    
    scorer = ScorePredictor()
    refiner = PoseRefinePredictor()
    glctx = dr.RasterizeCudaContext()
    est = FoundationPose(model_pts=mesh.vertices, model_normals=mesh.vertex_normals, mesh=mesh, scorer=scorer, refiner=refiner, debug_dir=debug_dir, debug=debug, glctx=glctx)
    logging.info("estimator initialization done")

    reader = YcbineoatReader(video_dir=args.test_scene_dir, shorter_side=None, zfar=np.inf)


    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)


    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)
    #print camera intrinsics
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    # print(intr)
    # model = YOLO("/home/mihawk/yolo/runs/segment/train8/weights/best.pt")
    model = YOLO("/home/mihawk/GPHT/Aza_train13_19Feb(11m)/weights/best.pt")
    
    # model = YOLO("/home/mihawk/Documents/Aza_train07_22J/train2/weights/epoch120.pt")
    i = 0
    # Streaming loop
    output = np.zeros((480, 640), np.uint8)
    
    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())/1e3
            color_image = np.asanyarray(color_frame.get_data())
            
            # color_image = cv2.imread("/home/mihawk/Cutie/dataset2/rgb/0240.png")
            # depth_image = cv2.imread("/home/mihawk/Cutie/dataset2/depth/0240.png",-1)/1e3
            depth_image[(depth_image<0.001) | (depth_image>=np.inf)] = 0
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            
            results = model(color_image, save=False, imgsz=640, conf=0.86)
            # logging.info(f'i:{i}')
            
            if len(results[0].boxes) != 0:
                find_bool = True
            else:
                find_bool = False  
                first_bool = True
                 
            output = reader.get_mask(0).astype(bool)
            # if find_bool:
            if find_bool and first_bool:
                # for boundingbox
                mask = results[0].boxes.xyxy[0]
                output = np.zeros((480, 640), np.uint8)
                cv2.rectangle(output, (int(mask[0]), int(mask[1])), (int(mask[2]), int(mask[3])), 255, -1)
                
                # for mask
                # mask = results[0].masks.xy[0]
                # pts = np.array(mask, np.int32)
                # pts = pts.reshape((-1, 1, 2))
                # output = np.zeros((480, 640), np.uint8)
                # cv2.fillPoly(output, [pts], 255)
                
                
                pose = est.register(K=reader.K, rgb=color_image, depth=depth_image, ob_mask=output, iteration=args.est_refine_iter)
                first_bool = False
            if find_bool and not first_bool:
                pose = est.track_one(rgb=color_image, depth=depth_image, K=reader.K, iteration=args.track_refine_iter)


                if debug>=1:
                    center_pose = pose@np.linalg.inv(to_origin)
                    print("-------------------")
                    print(center_pose)
                    print("-------------------")
                    print("pose:\n", pose)
                    # json_data = json.dumps([1, 2, 3, 4, 5])
                    json_data = json.dumps(center_pose.tolist())
                    sock.sendall(json_data.encode('utf-8'))
                    # vis = draw_posed_3d_box(reader.K, img=color_image, ob_in_cam=center_pose, bbox=bbox)
                    vis = draw_xyz_axis(color_image, ob_in_cam=center_pose, scale=0.1, K=reader.K, thickness=3, transparency=0)
                    cv2.imshow('1', vis)
            
            
            # images = np.hstack((color_image, depth_colormap))
            
            
            for result in results: 
                im = result.plot(conf = False)
                # concat = np.hstack((im, output))
                cv2.imshow('image', im)

            i += 1
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
            if key & 0xFF == ord('r'):
                first_bool = True
    finally:
        pipeline.stop()
        s.close()
                    