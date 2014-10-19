{-# LANGUAGE TemplateHaskell #-}
module Main where

import           Control.Applicative    ((<$>), (<*>))
import           Control.Lens           (view, (^.), _2)
import           Control.Lens.TH        (makeLenses)
import           Control.Monad          (msum, unless)
import           Control.Monad.Loops    (unfoldM)
import           Data.Function          (on)
import           Data.List              (maximumBy, minimumBy)
import           Data.Maybe             (fromJust, isNothing, maybeToList)
import           Data.Monoid            ((<>))
import           Data.Ord               (comparing)
import           Debug.Trace            (traceShowId)
import           Graphics.UI.SDL.Events (pollEvent)
import qualified Graphics.UI.SDL.Render as SDLR
import qualified Graphics.UI.SDL.Types  as SDLT
import           Linear.Matrix          ((!*))
import           Linear.Metric          (dot, quadrance, signorm)
import           Linear.V2              (V2 (..), perp, _x, _y)
import           Linear.V3              (V3 (..))
import           Linear.Vector          ((*^), (^*))
import           Physie.SDL             (drawLine, isQuitEvent, withImgInit,
                                         withRenderer, withWindow)

data Line = Line (V2 Float) (V2 Float) deriving Show

lineVector :: Line -> V2 Float
lineVector (Line a b) = b - a

lineStart :: Line -> V2 Float
lineStart (Line a _) = a

lineEnd :: Line -> V2 Float
lineEnd (Line _ b) = b

boolToList :: Bool -> a -> [a]
boolToList b a = [a | b]

maximumByNeighbors :: Ord a => (a -> a -> Ordering) -> [a] -> (a,a,a)
maximumByNeighbors f ls = let cls = cycle ls
                          in maximumBy (f `on` view _2) $ zip3 (drop (length ls - 1) cls) ls (drop 1 cls)

toIntPoint (V2 x y) = V2 (floor x) (floor y)

minmax :: Ord a => [a] -> (a,a)
minmax ls = (minimum ls,maximum ls)

cross2 :: V2 Float -> V2 Float -> Float
cross2 (V2 x1 y1) (V2 x2 y2) = x1 * y2 - y1 * x2

lineIntersection :: Float -> Line -> Line -> Maybe (V2 Float)
lineIntersection delta (Line p to1) (Line q to2)
  | collinear && overlapping = Just (p + ((tnom/denom) *^ r))
  | collinear && not overlapping = Nothing
  | abs denom <= delta && abs unom > delta = Nothing
  | abs denom > delta && isNormalized (tnom / denom) && isNormalized (unom / denom) = Just (p + (tnom/denom) *^ r)
  | otherwise = Nothing
  where r = to1 - p
        s = to2 - q
        denom = r `cross2` s
        tnom = (q - p) `cross2` s
        unom = (q - p) `cross2` r
        isNormalized z = z >= 0 && z <= 1
        collinear = abs denom <= delta && abs unom <= delta
        overlapping = (0 <= ((q - p) `dot` r) && ((q - p) `dot` r) <= (r `dot` r)) || (0 <= (p - q) `dot` s && (p - q) `dot` s <= s `dot` s)

extractMaybe :: (Maybe a,b) -> Maybe (a,b)
extractMaybe (Nothing,_) = Nothing
extractMaybe (Just a,b) = Just (a,b)

lineNormal :: Line -> V2 Float
lineNormal (Line a b) = perp .  signorm $ b - a


data Rectangle = Rectangle Float Float

data RigidBody = RigidBody {
    _bodyPosition        :: V2 Float
  , _bodyRotation        :: Float
  , _bodyLinearVelocity  :: V2 Float
  , _bodyAngularVelocity :: Float
  , _bodyMass            :: Maybe Float
  , _bodyShape           :: Rectangle
  }

$(makeLenses ''RigidBody)

data Collision = Collision {
    _collContactPoint :: V2 Float
  , _collNormal       :: V2 Float
  } deriving(Show)

$(makeLenses ''Collision)

rectanglePoints :: V2 Float -> Float -> Rectangle -> [V2 Float]
rectanglePoints (V2 x y) rot (Rectangle w h) = (to2 . (rmatrix !*) . to3) <$> points
  where hw = w/2
        hh = h/2
        points = [V2 (x-hw) (y-hh),V2 (x-hw) (y+hh),V2 (x+hw) (y+hh),V2 (x+hw) (y-hh)]
        r00 = cos rot
        r01 = -(sin rot)
        r10 = sin rot
        r11 = (cos rot)
        rmatrix = V3 (V3 r00 r01 (x-r00 * x - r01 * y)) (V3 r10 r11 (y - r10 * x - r11 * y)) (V3 0 0 1)
        to3 (V2 x' y') = V3 x' y' 1
        to2 (V3 x' y' _) = V2 x' y'

rectangleLines :: V2 Float -> Float -> Rectangle -> [Line]
rectangleLines pos rot rect = zipWith Line <*> (tail . cycle) $ rectanglePoints pos rot rect

detectCollision :: RigidBody -> RigidBody -> Maybe Collision
detectCollision b1 b2 = msum $ (((uncurry Collision) <$>) . extractMaybe) <$> [(lineIntersection 0.001 x y,lineNormal x) | x <- bodyLines b1,y <- bodyLines b2]

bodyLines :: RigidBody -> [Line]
bodyLines b1 = rectangleLines (b1 ^. bodyPosition) (b1 ^. bodyRotation) (b1 ^. bodyShape)

bodyPoints :: RigidBody -> [V2 Float]
bodyPoints b1 = rectanglePoints (b1 ^. bodyPosition) (b1 ^. bodyRotation) (b1 ^. bodyShape)

screenWidth :: Int
screenWidth = 640

screenHeight :: Int
screenHeight = 480

drawBody :: SDLT.Renderer -> RigidBody -> IO ()
drawBody r b = mapM_ (drawLine r) ((\(Line x y) -> (floor <$> x,floor <$> y)) <$> (bodyLines b))

findBestEdge :: [V2 Float] -> V2 Float -> (V2 Float,Line)
findBestEdge ls n = let (v0,v,v1) = maximumByNeighbors (comparing (n `dot`)) ls
                    in if (v - v0) `dot` n <= (v - v1) `dot` n
                       then (v,Line v0 v)
                       else (v,Line v v1)

clip :: V2 Float -> V2 Float -> V2 Float -> Float -> [V2 Float]
clip v1 v2 n o = let d1 = n `dot` v1 - o
                     d2 = n `dot` v2 - o
                     e = v2 - v1
                 in boolToList (d1 >= 0) v1 <>
                    boolToList (d2 >= 0) v2 <>
                    boolToList (d1 * d2 < 0) (v1 + (d1 / (d1 - d2)) *^ e)

findContactPoints :: [V2 Float] -> [V2 Float] -> V2 Float -> [V2 Float]
findContactPoints a b n =
  let e1 = traceShowId $ findBestEdge a n
      e2 = traceShowId $ findBestEdge b (-n)
      e1Smaller = abs (lineVector (snd e1) `dot` n) <= abs (lineVector (snd e2) `dot` n)
      ref = if e1Smaller then e1 else e2
      inc = if e1Smaller then e2 else e1
      nref = (signorm . lineVector . snd) ref
      o1 = nref `dot` lineStart (snd ref)
      [cp0,cp1] = clip (lineStart (snd inc)) (lineEnd (snd inc)) nref o1
      o2 = nref `dot` lineEnd (snd ref)
      [cp2,cp3] = clip cp0 cp1 (negate nref) (-o2)
      refNorm = (if e1Smaller then 1 else -1) *^ V2 (negate $ nref ^. _y) (nref ^. _x)
      refNormMax = refNorm `dot` fst ref
  in  boolToList (refNorm `dot` cp2 - refNormMax >= 0) cp2 <>
      boolToList (refNorm `dot` cp3 - refNormMax >= 0) cp3

satIntersectsBodies :: RigidBody -> RigidBody -> Maybe (V2 Float)
satIntersectsBodies a b = case satIntersects (bodyLines a) (bodyLines b) of
  Nothing -> Nothing
  Just p -> Just $ if ((a ^. bodyPosition) - (b ^. bodyPosition)) `dot` p < 0
                   then negate p
                   else p

-- Courtesy of http://elancev.name/oliver/2D%20polygon.htm
satIntersects :: [Line] -> [Line] -> Maybe (V2 Float)
satIntersects a b = let axes = concatMap (map (perp . lineToVector)) [a,b]
                        separationData = map (separates a b) axes
                    in if any isNothing separationData
                       then Nothing
                       else Just $ minimumBy (compare `on` quadrance) $ concatMap maybeToList separationData
  where lineToVector (Line x y) = y - x
        calculateInterval axis = minmax . map ((axis `dot`) . lineToVector)
        separates x y axis = let (mina,maxa) = calculateInterval axis x
                                 (minb,maxb) = calculateInterval axis y
                             in if mina > maxb || minb > maxa
                                then Nothing
                                else let d0 = maxa - minb
                                         d1 = maxb - mina
                                         depth = if d0 < d1 then d0 else d1
                                     in Just $ axis ^* (depth / (quadrance axis))

findSupportPoints :: V2 Float -> [V2 Float] -> [V2 Float]
findSupportPoints n vs = let dots = (`dot` n) <$> vs
                             mindot = minimum dots
                         in take 2 . map snd . filter (\(d,_) -> d < mindot + 0.001) $ zip dots vs

clamp :: Ord a => a -> a -> a -> a
clamp minv maxv = min maxv . max minv

findContactPoint :: [V2 Float] -> [V2 Float] -> V2 Float
findContactPoint [x] [_,_] = x
findContactPoint [a,b] [x] = let av = x - a
                                 ab = b - a
                                 t = clamp 0 1 $ (av `dot` ab) / (ab `dot` ab)
                             in a + t *^ ab
findContactPoint [a,b] [x,y] = findContactPoint [a,b] [x]

mainLoop :: SDLT.Renderer -> Float -> IO ()
mainLoop renderer angle = do
  events <- unfoldM pollEvent
  SDLR.setRenderDrawColor renderer 0 0 0 255
  SDLR.renderClear renderer
  SDLR.setRenderDrawColor renderer 255 255 255 255
  let body1 = RigidBody (V2 100 100) 0 (V2 0 0) 0 Nothing (Rectangle 100 100)
      body2 = RigidBody (V2 150 150) 0 (V2 0 0) 0 Nothing (Rectangle 50 50)
  drawBody renderer body1
  drawBody renderer body2
  let n1 = signorm $ fromJust $ satIntersectsBodies body1 body2
  let n2 = signorm $ fromJust $ satIntersectsBodies body2 body1
  let sp1 = findSupportPoints n1 (bodyPoints body1)
  let sp2 = findSupportPoints n2 (bodyPoints body2)
  let cp1 = findContactPoint sp1 sp2
  let cp2 = findContactPoint sp2 sp1
  SDLR.setRenderDrawColor renderer 255 0 0 255
  drawLine renderer (toIntPoint cp1,toIntPoint $ cp1 + 10 *^ n1)
  drawLine renderer (toIntPoint cp2,toIntPoint $ cp2 + 10 *^ n2)
  --mapM_ (\(x,y) -> drawLine renderer (toIntPoint x,toIntPoint y)) $ ((\p -> (p,p+(10 *^ n1))) <$> sp1) ++ ((\p -> (p,p+(10 *^ n2))) <$> sp2)
  --drawLine renderer (toIntPoint sp2,toIntPoint (sp2 + 10 *^ n2))
--  let collision = detectCollision body1 body2
--  SDLR.setRenderDrawColor renderer 255 0 0 255
--  case collision of
--   Nothing -> return ()
--   Just (Collision contactPoint normal) ->
--     drawLine renderer (toIntPoint contactPoint,toIntPoint $ contactPoint + (normal ^* 10))
  SDLR.renderPresent renderer
  unless (any isQuitEvent events) $ mainLoop renderer (angle + 0.001)

main :: IO ()
main = do
  print $ findContactPoints [V2 4 2,V2 12 2,V2 12 5,V2 4 5] [V2 8 4,V2 14 4,V2 14 9,V2 8 14] (V2 0 (-1))
--  let body1 = RigidBody (V2 100 100) 0 (V2 0 0) 0 Nothing (Rectangle 100 100)
--      body2 = RigidBody (V2 150 150) 0 (V2 0 0) 0 Nothing (Rectangle 50 50)
--  let n1 = signorm $ fromJust $ satIntersectsBodies body1 body2
--  let n2 = signorm $ fromJust $ satIntersectsBodies body2 body1
--  print n1
--  print $ findSupportPoints n1 (bodyPoints body1)
--  print $ findSupportPoints n2 (bodyPoints body2)
--    print $ detectCollision (RigidBody (V2 0 0) 0.1 (V2 0 0) 0 Nothing (Rectangle 10 10)) (RigidBody (V2 5 5) 0 (V2 0 0) 0 Nothing (Rectangle 5 5))
 --   print $ setIntersects () ()
  --let firstLines = bodyLines $ RigidBody (V2 100 100) 0.1 (V2 0 0) 0 Nothing (Rectangle 100 100)
  --let secondLines = bodyLines $ RigidBody (V2 150 150) 2.5 (V2 0 0) 0 Nothing (Rectangle 50 50)
  --print $ satIntersects firstLines secondLines
  --print $ satIntersects secondLines firstLines
  withImgInit $
    withWindow "racie 0.0.1.1" $ \window ->
      withRenderer window screenWidth screenHeight (`mainLoop` 0)
