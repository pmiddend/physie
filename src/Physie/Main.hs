{-# LANGUAGE TemplateHaskell #-}
module Main where

import Physie.SDL(withImgInit,withWindow,withRenderer,drawLine,isQuitEvent)
import qualified Graphics.UI.SDL.Render as SDLR
import qualified Graphics.UI.SDL.Types as SDLT
import           Graphics.UI.SDL.Events     (pollEvent)
import Control.Monad.Loops(unfoldM)
import Control.Lens((^.))
import Linear.V2(V2(..),perp)
import Linear.V3(V3(..))
import Linear.Matrix((!*))
import Linear.Vector((*^),(^*))
import Linear.Metric(dot,signorm,quadrance)
import Control.Lens.TH(makeLenses)
import Control.Monad(msum)
import Control.Applicative((<$>),(<*>))
import Data.Maybe(fromJust,isNothing,maybeToList)
import Data.List(minimumBy)
import Data.Function(on)

data Rectangle = Rectangle Float Float

data RigidBody = RigidBody {
    _bodyPosition :: V2 Float
  , _bodyRotation :: Float
  , _bodyLinearVelocity :: V2 Float
  , _bodyAngularVelocity :: Float
  , _bodyMass :: Maybe Float
  , _bodyShape :: Rectangle
  }

$(makeLenses ''RigidBody)

data Collision = Collision {
    _collContactPoint :: V2 Float
  , _collNormal :: V2 Float
  } deriving(Show)

$(makeLenses ''Collision)

data Line = Line (V2 Float) (V2 Float)

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

toIntPoint (V2 x y) = V2 (floor x) (floor y)

minmax :: Ord a => [a] -> (a,a)
minmax ls = (minimum ls,maximum ls)

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
findContactPoint _ _ = error "not defined"

mainLoop :: SDLT.Renderer -> Float -> IO ()
mainLoop renderer angle = do
  events <- unfoldM pollEvent
  SDLR.setRenderDrawColor renderer 0 0 0 255
  SDLR.renderClear renderer
  SDLR.setRenderDrawColor renderer 255 255 255 255
  let body1 = RigidBody (V2 100 100) 0.1 (V2 0 0) 0 Nothing (Rectangle 100 100)
      body2 = RigidBody (V2 150 150) 2.5 (V2 0 0) 0 Nothing (Rectangle 50 50)
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
  if any isQuitEvent events
     then return ()
     else mainLoop renderer (angle + 0.001)

main :: IO ()
main = do
  let body1 = RigidBody (V2 100 100) 0.1 (V2 0 0) 0 Nothing (Rectangle 100 100)
      body2 = RigidBody (V2 150 150) 2.5 (V2 0 0) 0 Nothing (Rectangle 50 50)
  let n1 = signorm $ fromJust $ satIntersectsBodies body1 body2
  let n2 = signorm $ fromJust $ satIntersectsBodies body2 body1
  print $ findSupportPoints n1 (bodyPoints body1)
  print $ findSupportPoints n2 (bodyPoints body2)
--    print $ detectCollision (RigidBody (V2 0 0) 0.1 (V2 0 0) 0 Nothing (Rectangle 10 10)) (RigidBody (V2 5 5) 0 (V2 0 0) 0 Nothing (Rectangle 5 5))
 --   print $ setIntersects () ()
  --let firstLines = bodyLines $ RigidBody (V2 100 100) 0.1 (V2 0 0) 0 Nothing (Rectangle 100 100)
  --let secondLines = bodyLines $ RigidBody (V2 150 150) 2.5 (V2 0 0) 0 Nothing (Rectangle 50 50)
  --print $ satIntersects firstLines secondLines
  --print $ satIntersects secondLines firstLines
  withImgInit $ do
    withWindow "racie 0.0.1.1" $ \window -> do
      withRenderer window screenWidth screenHeight (\r -> mainLoop r 0)
