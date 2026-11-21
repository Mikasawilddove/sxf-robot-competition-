"""Image-face proposals for visualization; never used as a grasp decision."""
import math
import cv2
import numpy as np
from suat_mission.vision.experimental_vision import ExperimentalVision
from suat_mission.vision.template_matcher import _iou

def _box(d):
    return [float(d.x-d.width/2),float(d.y-d.height/2),
            float(d.x+d.width/2),float(d.y+d.height/2)]

def _inside(box, width, height):
    return [max(0.,min(width-1.,box[0])),max(0.,min(height-1.,box[1])),
            max(0.,min(width-1.,box[2])),max(0.,min(height-1.,box[3]))]

class RotatedTemplates:
    """Eight rotated template variants; scores are normalized correlations."""
    def __init__(self, images, thresholds=None, scales=None, angles=None, image_scale=.5):
        self.factor=float(image_scale)
        if not .25 <= self.factor <= 1.:
            raise ValueError('invalid image scale')
        self.thresholds=thresholds or {'teddybear':.72,'clock':.60,'cake':.72}
        self.variants=[]
        scales=scales or [x/10. for x in range(4,15)]
        angles=angles or list(range(0,360,45))
        for label in ('teddybear','clock','cake'):
            for img in images[label]:
                if img is None or img.size==0: raise ValueError('missing template '+label)
                gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) if img.ndim==3 else img
                h,w=gray.shape
                for angle in angles:
                    if angle%90==0:
                        rotated=np.ascontiguousarray(np.rot90(gray,int(angle//90)))
                    else:
                        side=int(math.ceil(math.hypot(w,h)))
                        matrix=cv2.getRotationMatrix2D((w/2.,h/2.),angle,1.)
                        matrix[0,2]+=(side-w)/2.;matrix[1,2]+=(side-h)/2.
                        rotated=cv2.warpAffine(gray,matrix,(side,side),flags=cv2.INTER_LINEAR,
                                               borderMode=cv2.BORDER_CONSTANT,borderValue=255)
                    for scale in scales:
                        tw=max(2,int(round(rotated.shape[1]*scale*self.factor)))
                        th=max(2,int(round(rotated.shape[0]*scale*self.factor)))
                        template=cv2.resize(rotated,(tw,th),interpolation=cv2.INTER_AREA)
                        if np.std(template)<3:continue
                        self.variants.append((label,angle,np.ascontiguousarray(template)))

    def detect(self,image):
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        if self.factor!=1:
            gray=cv2.resize(gray,None,fx=self.factor,fy=self.factor,interpolation=cv2.INTER_AREA)
        found=[]
        for label,angle,template in self.variants:
            h,w=template.shape
            if h>gray.shape[0] or w>gray.shape[1]:continue
            scores=cv2.matchTemplate(gray,template,cv2.TM_CCOEFF_NORMED)
            # Rotated square padding can resemble other classes. Non-quarter
            # rotations require stronger evidence until a field dataset exists.
            threshold=self.thresholds[label] if angle%90==0 else max(self.thresholds[label],.78)
            local=(scores>=threshold) & (scores==cv2.dilate(scores,np.ones((3,3),np.uint8)))
            ys,xs=np.where(local)
            if len(xs)>15:
                keep=np.argpartition(scores[ys,xs],-15)[-15:];xs,ys=xs[keep],ys[keep]
            for x,y in zip(xs,ys):
                f=self.factor
                found.append({'label':label,'score':float(scores[y,x]),'method':'rotated_template',
                              'angle_deg':angle,'partial':False,
                              'box':[float(x/f),float(y/f),float((x+w)/f),float((y+h)/f)]})
        found.sort(key=lambda x:x['score'],reverse=True)
        chosen=[]
        for item in found:
            if any(item['label']==d['label'] and _iou(item['box'],d['box'])>.3 for d in chosen):continue
            chosen.append(item)
        return chosen

class MultiViewPreview:
    """Visual proposal generator: not calibrated probability, pose, or grasp output."""
    def __init__(self, templates):
        self.feature=ExperimentalVision(templates,banana_saturation_min=55)
        self.rotated=RotatedTemplates(templates)
    def detect(self,image):
        if image is None or not image.size: raise ValueError('empty image')
        h,w=image.shape[:2]
        base=[{'label':d.label,'score':float(d.score),'method':'feature_or_color',
               'angle_deg':None,'partial':False,'box':_box(d)} for d in self.feature.detect(image)]
        extra=self.rotated.detect(image)
        result=list(base)
        for item in extra:
            if any(item['label']==d['label'] and _iou(item['box'],d['box'])>.25 for d in result):
                continue
            result.append(item)
        # A clipped feature proposal is displayed only and never used for grasping.
        partial=self._partial_features(image,{'cake','clock','teddybear'})
        for item in partial:
            if any(item['label']==d['label'] and _iou(item['box'],d['box'])>.25 for d in result):
                continue
            result.append(item)
        return result

    def _partial_features(self,image,labels):
        h,w=image.shape[:2]
        keys,desc=self.feature.brisk.detectAndCompute(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),None)
        if desc is None:return []
        result=[]
        for label,shape,ref,rd in self.feature.refs:
            if label not in labels:continue
            pairs=self.feature.matcher.knnMatch(rd,desc,k=2)
            good=[p[0] for p in pairs if len(p)==2 and p[0].distance<.8*p[1].distance]
            if len(good)<8:continue
            src=np.float32([ref[m.queryIdx].pt for m in good])
            dst=np.float32([keys[m.trainIdx].pt for m in good])
            matrix,mask=cv2.findHomography(src,dst,cv2.RANSAC,3.)
            if matrix is None or mask is None or not np.isfinite(matrix).all():continue
            inliers=int(mask.sum()); score=inliers/float(len(good))
            if inliers<8 or score<.45:continue
            rh,rw=shape
            quad=cv2.perspectiveTransform(np.float32([[0,0],[rw,0],[rw,rh],[0,rh]]).reshape(-1,1,2),matrix).reshape(-1,2)
            if not np.isfinite(quad).all() or not cv2.isContourConvex(quad.astype(np.float32)):continue
            area=abs(cv2.contourArea(quad.astype(np.float32)))
            if area<100 or area>h*w*.5:continue
            if not (quad[:,0].min()<0 or quad[:,1].min()<0 or quad[:,0].max()>=w or quad[:,1].max()>=h):continue
            frame=np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]])
            overlap,_=cv2.intersectConvexConvex(quad.astype(np.float32),frame)
            if overlap/area<.65:continue
            box=_inside([float(quad[:,0].min()),float(quad[:,1].min()),
                         float(quad[:,0].max()),float(quad[:,1].max())],w,h)
            if box[2]<=box[0] or box[3]<=box[1]:continue
            candidate={'label':label,'score':float(score),'method':'partial_feature',
                       'angle_deg':None,'partial':True,'box':box}
            if any(d['label']==label and _iou(box,d['box'])>.3 for d in result):continue
            result.append(candidate)
        return result

class FaceStability:
    """Short-lived box overlap evidence; not a physical-object identity."""
    def __init__(self,max_age=2.5,min_iou=.3):
        self.max_age=float(max_age);self.min_iou=float(min_iou);self.tracks=[]
    def update(self,detections,now):
        live=[d for d in self.tracks if 0<=now-d['time']<=self.max_age]
        used=set();new=[];output=[]
        for item in detections:
            choices=[(_iou(item['box'],prior['box']),i,prior) for i,prior in enumerate(live)
                     if i not in used and prior['label']==item['label']
                     and prior['partial']==item['partial']]
            best=max(choices,default=(0.,None,None),key=lambda t:t[0])
            if best[0]>=self.min_iou:
                used.add(best[1]);count=best[2]['count']+1
            else:count=1
            value=dict(item,support_frames=count)
            output.append(value)
            new.append({'label':item['label'],'partial':item['partial'],
                        'box':list(item['box']),'count':count,'time':now})
        self.tracks=new+[prior for i,prior in enumerate(live) if i not in used]
        return output
