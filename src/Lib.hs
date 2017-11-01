--{-# LANGUAGE OverloadedStrings #-}

module Lib
    ( someFunc
    , readConfiguration
    ) where

import qualified Debug.Trace as Debug
import qualified Data.Maybe as Maybe
import qualified Data.List as List

import qualified System.Random as Random

import qualified Data.Aeson as Aeson

import Linear.V3 as V3
import Linear.Matrix as Matrix
import Linear.Vector
import Linear.Metric

import qualified Data.ByteString as ByteString
import qualified Data.ByteString.Lazy as ByteString.Lazy

import qualified Graphics.Gloss as Gloss
import qualified Graphics.Gloss.Interface.IO.Game as Game

data Limb = Limb
    { limbLength :: Double
    , limbAngle :: Double
    }

limbStateToString :: Limb -> String
limbStateToString limb =
    "{\"angle\":" ++ show (limbAngle limb) ++ ", \"length\":" ++ show (limbLength limb) ++ "}"


limbTransformMatrix :: Limb -> Matrix.M33 Double
limbTransformMatrix limb =
    let
        angle = limbAngle limb

        rotation =
            V3  (V3 (cos angle) (-(sin angle)) 0)
                (V3 (sin angle)   (cos angle) 0)
                (V3 0 0 1)
        translation =
            V3  (V3 1 0 (limbLength limb))
                (V3 0 1 0)
                (V3 0 0 1)
    in
        rotation !*! translation


limbTransformationChain :: [Limb] -> [Matrix.M33 Double]
limbTransformationChain limbs =
    let
        foldFunction (accChain, transformMatrix) limb =
            let
                newTransformMatrix = transformMatrix !*! (limbTransformMatrix limb)
            in
                (accChain ++ [newTransformMatrix], newTransformMatrix)

        (chain, transformMatrix) =
            List.foldl foldFunction ([], Matrix.identity) limbs
    in
        chain



finalPoint :: [Limb] -> V3 Double
finalPoint limbs =
    let
        transformMatrix =
            List.foldl
                (!*!)
                Matrix.identity
                $ List.map limbTransformMatrix limbs
    in
        transformMatrix !* (V3 0 0 1)


gradientDescentStep :: [Double] -> ([Double] -> t) -> (t -> Double) -> [Double]
gradientDescentStep vars valueFunction errorFunction =
    let
        delta = 0.001
        learningRate = 0.0001

        nthDelta n =
            (List.replicate n 0) ++ [delta] ++ (List.repeat 0)

        deltaLists =
            List.map nthDelta $ List.iterate (+ 1) 0

        deltaAddFunc delta =
            List.zipWith (+) vars delta

        varsWithDelta =
            List.map (deltaAddFunc) $ deltaLists

        currentError =
            errorFunction $ valueFunction vars

        gradients =
            List.map
                (\withDelta ->
                    ((errorFunction $ valueFunction withDelta) - currentError) / delta)
                varsWithDelta
    in
        List.zipWith (-) vars $ List.map ((*) learningRate) gradients


gradientDescent :: [Double] -> ([Double] -> t) -> (t -> Double) -> [Double]
gradientDescent initialVars  valueFunction  errorFunction =
    let
        stopCondition oldError newError =
            newError - oldError < 0.001

        currError =
            errorFunction $ valueFunction initialVars

        newVars =
            gradientDescentStep initialVars valueFunction errorFunction

        newError =
            errorFunction $ valueFunction newVars
    in
        if stopCondition currError newError then
            newVars
        else
            gradientDescent newVars valueFunction errorFunction


inverseKinematics :: [Limb] -> V3 Double -> [Limb]
inverseKinematics limbs target =
    let
        initialAngles =
            fmap limbAngle limbs

        lengths =
            fmap limbLength limbs

        valueFunction angles =
            finalPoint $ List.zipWith Limb lengths angles

        errorFunction point =
            Linear.Metric.norm (point ^-^ target)

        newAngles =
            gradientDescent initialAngles valueFunction errorFunction
    in
        List.zipWith Limb lengths newAngles


readConfiguration :: IO (Maybe [Double])
readConfiguration =
    do
        -- Read the limb legs
        legAnglesStr <- ByteString.getLine
        return $ Aeson.decode $ ByteString.Lazy.fromStrict legAnglesStr


readTarget :: IO (Maybe (V3 Double))
readTarget =
    do
        targetStr <- ByteString.getLine
        let tuple = Aeson.decode (ByteString.Lazy.fromStrict targetStr) :: Maybe (Double, Double)
        return $ (\(x,y) -> V3 x y 1) <$> tuple




drawLimbPositions :: [V3 Double] -> Gloss.Picture
drawLimbPositions limbPositions =
    let
        drawFunction (V3 x y _) =
            Gloss.translate (realToFrac x) (realToFrac y) $ Gloss.Circle(5)

        pictures =
            List.map drawFunction limbPositions
    in
        Gloss.pictures pictures

drawLimbArms :: [V3 Double] -> Gloss.Picture
drawLimbArms limbPositions =
    Gloss.line
        $ fmap (\(V3 x y _) -> (realToFrac x, realToFrac y)) $ [V3 0 0 1] ++ limbPositions


drawLimbs :: [Limb] -> Gloss.Picture
drawLimbs limbs =
    let
        positions =
            List.map (!* (V3 0 0 1))
                $ limbTransformationChain limbs
    in
        Gloss.pictures
            [ drawLimbPositions positions
            , drawLimbArms positions
            ]


eventHandler :: Game.Event -> [Limb] -> [Limb]
eventHandler event oldLimbs =
    case event of
        Game.EventMotion (x,y) ->
            inverseKinematics
                oldLimbs
                (V3 (realToFrac x) (realToFrac y) 1)
        _ ->
            oldLimbs


someFunc :: IO ()
someFunc =
    let
        initialLimbs =
            List.replicate 10 (Limb 30 0)

        lines =
            Gloss.pictures
                [ Gloss.line [(-2000, 0), (2000, 0)]
                , Gloss.line [(0, -2000), (0, 2000)]
                ]

        pictureFunc limbs =
            Gloss.pictures
                [ drawLimbs limbs
                , lines
                ]
    in
    do
        r1 <- Random.getStdGen
        let randList = Random.randomRs (0, maxBound) r1 :: [Int]
        Gloss.play
            (Gloss.InWindow "Window" (200, 200) (10, 10))
            Gloss.white
            1000
            initialLimbs
            pictureFunc
            eventHandler
            (\_ limbs -> limbs)

