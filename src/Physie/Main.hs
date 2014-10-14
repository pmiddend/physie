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
import Linear.Matrix((!*),inv33)
import Linear.Vector((*^))
import Linear.Metric(dot,signorm)
import Control.Lens.TH(makeLenses)
import Control.Monad(msum)
import Control.Applicative((<$>),(<*>))
import Data.Maybe(fromJust)

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
  }

$(makeLenses ''Collision)

data Line = Line (V2 Float) (V2 Float)

rectangleLines :: V2 Float -> Float -> Rectangle -> [Line]
rectangleLines (V2 x y) rot (Rectangle w h) = zipWith Line <*> (tail . cycle) $ rpoints
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
        rpoints = (to2 . (rmatrix !*) . to3) <$> points

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
detectCollision b1 b2 = msum $ (((uncurry Collision) <$>) . extractMaybe) <$> [(lineIntersection 0.001 x y,lineNormal x) | x <- rectangleLines (b1 ^. bodyPosition) (b1 ^. bodyRotation) (b1 ^. bodyShape),y <- rectangleLines (b2 ^. bodyPosition) (b2 ^. bodyRotation) (b2 ^. bodyShape)]

screenWidth :: Int
screenWidth = 640

screenHeight :: Int
screenHeight = 480

mainLoop :: SDLT.Renderer -> Float -> IO ()
mainLoop renderer angle = do
  events <- unfoldM pollEvent
  SDLR.setRenderDrawColor renderer 0 0 0 255
  SDLR.renderClear renderer
  SDLR.setRenderDrawColor renderer 255 255 255 255
  mapM_ (drawLine renderer) ((\(Line x y) -> (floor <$> x,floor <$> y)) <$> (rectangleLines (V2 200 250) angle (Rectangle 50 100)))
  SDLR.renderPresent renderer
  if any isQuitEvent events
     then return ()
     else mainLoop renderer (angle + 0.001)

main :: IO ()
main = do
    withImgInit $ do
      withWindow "racie 0.0.1.1" $ \window -> do
        withRenderer window screenWidth screenHeight (\r -> mainLoop r 0)
