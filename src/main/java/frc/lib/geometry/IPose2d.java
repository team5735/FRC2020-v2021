package frc.lib.geometry;

public interface IPose2d<S> extends IRotation2d<S>, ITranslation2d<S> {
    public Pose getPose();

    public S transformBy(Pose transform);

    public S mirror();
}
